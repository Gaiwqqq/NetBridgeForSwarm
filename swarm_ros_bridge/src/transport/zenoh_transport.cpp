#include "transport/zenoh_transport.hpp"

#include <zenoh.hxx>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace swarm_ros_bridge {
namespace transport {
namespace {

void SetError(const std::string& value, std::string* error) {
  if (error != nullptr) {
    *error = value;
  }
}

bool IsTimeoutError(const std::string& value) {
  std::string normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return normalized.find("timeout") != std::string::npos ||
         normalized.find("timed out") != std::string::npos;
}

std::string JsonString(const std::string& value) {
  std::string result{"\""};
  for (const char ch : value) {
    if (ch == '\\' || ch == '\"') {
      result.push_back('\\');
    }
    result.push_back(ch);
  }
  result.push_back('\"');
  return result;
}

std::string JsonArray(const std::vector<std::string>& values) {
  std::ostringstream stream;
  stream << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0U) {
      stream << ',';
    }
    stream << JsonString(values[i]);
  }
  stream << ']';
  return stream.str();
}

zenoh::Priority ToZenohPriority(QosClass qos_class) {
  switch (qos_class) {
    case QosClass::kCommand:
      return Z_PRIORITY_REAL_TIME;
    case QosClass::kState:
      return Z_PRIORITY_DATA_HIGH;
    case QosClass::kBulk:
      return Z_PRIORITY_DATA_LOW;
    case QosClass::kService:
      return Z_PRIORITY_INTERACTIVE_HIGH;
  }
  return Z_PRIORITY_DEFAULT;
}

class BoundedExecutor {
 public:
  BoundedExecutor(std::size_t thread_count, std::size_t capacity)
      : capacity_(std::max<std::size_t>(1U, capacity)) {
    const std::size_t count = std::max<std::size_t>(1U, thread_count);
    workers_.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      workers_.emplace_back([this]() { Run(); });
    }
  }

  ~BoundedExecutor() { Stop(); }

  bool TrySubmit(std::function<void()> task) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || tasks_.size() >= capacity_) {
      return false;
    }
    tasks_.push_back(std::move(task));
    condition_.notify_one();
    return true;
  }

  void Stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) {
        return;
      }
      stopping_ = true;
    }
    condition_.notify_all();
    for (std::thread& worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    workers_.clear();
  }

 private:
  void Run() {
    while (true) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() { return stopping_ || !tasks_.empty(); });
        if (stopping_ && tasks_.empty()) {
          return;
        }
        task = std::move(tasks_.front());
        tasks_.pop_front();
      }
      try {
        task();
      } catch (...) {
        // Individual query tasks own their error replies; never terminate a worker.
      }
    }
  }

  const std::size_t capacity_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<std::function<void()>> tasks_;
  std::vector<std::thread> workers_;
  bool stopping_{false};
};

}  // namespace

class ZenohTransport::Impl : public std::enable_shared_from_this<ZenohTransport::Impl> {
 public:
  Impl(const ZenohTransportConfig& config, const std::string& hostname)
      : executor_(config.service_worker_threads, config.service_queue_capacity) {
    auto zenoh_config = zenoh::Config::create_default();
    zenoh_config.insert_json5("mode", JsonString(config.mode));
    zenoh_config.insert_json5("scouting/multicast/enabled",
                              config.multicast_scouting ? "true" : "false");
    zenoh_config.insert_json5("scouting/gossip/enabled",
                              config.gossip_scouting ? "true" : "false");
    if (!config.listen_endpoints.empty()) {
      zenoh_config.insert_json5("listen/endpoints", JsonArray(config.listen_endpoints));
    }
    if (!config.connect_endpoints.empty()) {
      zenoh_config.insert_json5("connect/endpoints", JsonArray(config.connect_endpoints));
    }
    zenoh_config.insert_json5("transport/unicast/compression/enabled",
                              config.compression_enabled ? "true" : "false");
    zenoh_config.insert_json5("transport/multicast/compression/enabled",
                              config.compression_enabled ? "true" : "false");

    session_ = std::make_unique<zenoh::Session>(
        zenoh::Session::open(std::move(zenoh_config)));
    zenoh::Session::LivelinessSubscriberOptions presence_options;
    presence_options.history = true;
    liveliness_subscriber_ = std::make_unique<zenoh::Subscriber<void>>(
        session_->liveliness_declare_subscriber(
            zenoh::KeyExpr("netbridge/v1/alive/*"),
            [this](const zenoh::Sample& sample) {
              std::string discovered_hostname;
              if (!ParseAliveKey(
                      std::string(sample.get_keyexpr().as_string_view()),
                      &discovered_hostname)) {
                return;
              }
              if (sample.get_kind() == Z_SAMPLE_KIND_PUT) {
                UpdatePresence(discovered_hostname, true);
              } else if (sample.get_kind() == Z_SAMPLE_KIND_DELETE) {
                UpdatePresence(discovered_hostname, false);
              }
            },
            zenoh::closures::none, std::move(presence_options)));
    UpdatePresence(hostname, true);
    liveliness_token_ = std::make_unique<zenoh::LivelinessToken>(
        session_->liveliness_declare_token(zenoh::KeyExpr(AliveKey(hostname))));
    session_open_.store(true);
  }

  ~Impl() { Close(); }

  void Close() {
    bool expected = false;
    if (!closed_.compare_exchange_strong(expected, true)) {
      return;
    }
    executor_.Stop();
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      liveliness_subscriber_.reset();
      liveliness_token_.reset();
      if (session_ != nullptr) {
        try {
          session_->close();
        } catch (...) {
        }
        session_.reset();
      }
      session_open_.store(false);
    }
  }

  zenoh::Session& Session() {
    if (closed_.load() || session_ == nullptr) {
      throw std::runtime_error("Zenoh session is closed");
    }
    return *session_;
  }

  TransportStats Stats() const {
    TransportStats stats;
    stats.session_open = session_open_.load();
    if (stats.session_open) {
      std::lock_guard<std::mutex> lock(session_mutex_);
      if (session_ != nullptr) {
        try {
          stats.connected_peer_count = session_->get_peers_z_id().size();
          stats.connected_router_count = session_->get_routers_z_id().size();
        } catch (...) {
        }
      }
    }
    stats.link_connected =
        stats.connected_peer_count + stats.connected_router_count > 0U;
    const bool was_connected = last_link_connected_.exchange(stats.link_connected);
    if (stats.link_connected && !was_connected) {
      if (ever_link_connected_.exchange(true)) {
        reconnect_count_.fetch_add(1U);
      }
    }
    stats.reconnect_count = reconnect_count_.load();
    stats.receive_decode_errors = receive_decode_errors_.load();
    stats.callback_drops = callback_drops_.load();
    stats.service_timeouts = service_timeouts_.load();
    stats.service_queue_drops = service_queue_drops_.load();
    return stats;
  }

  std::vector<NodePresence> Presence() const {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(presence_mutex_);
    std::vector<NodePresence> nodes;
    nodes.reserve(presence_.size());
    for (const auto& item : presence_) {
      NodePresence node;
      node.hostname = item.first;
      node.online = item.second.online;
      node.state_age_ms = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              now - item.second.state_changed_at)
              .count());
      node.online_transitions = item.second.online_transitions;
      nodes.push_back(std::move(node));
    }
    return nodes;
  }

  BoundedExecutor executor_;
  mutable std::mutex session_mutex_;
  std::unique_ptr<zenoh::Session> session_;
  std::unique_ptr<zenoh::Subscriber<void>> liveliness_subscriber_;
  std::unique_ptr<zenoh::LivelinessToken> liveliness_token_;
  std::atomic<bool> session_open_{false};
  std::atomic<bool> closed_{false};
  mutable std::atomic<bool> last_link_connected_{false};
  mutable std::atomic<bool> ever_link_connected_{false};
  mutable std::atomic<std::uint64_t> reconnect_count_{0};
  std::atomic<std::uint64_t> receive_decode_errors_{0};
  std::atomic<std::uint64_t> callback_drops_{0};
  std::atomic<std::uint64_t> service_timeouts_{0};
  std::atomic<std::uint64_t> service_queue_drops_{0};

 private:
  struct PresenceRecord {
    bool online{false};
    std::chrono::steady_clock::time_point state_changed_at;
    std::uint64_t online_transitions{0};
  };

  void UpdatePresence(const std::string& hostname, bool online) {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(presence_mutex_);
    const auto existing = presence_.find(hostname);
    if (existing == presence_.end()) {
      PresenceRecord record;
      record.online = online;
      record.state_changed_at = now;
      record.online_transitions = online ? 1U : 0U;
      presence_.emplace(hostname, std::move(record));
      return;
    }
    if (existing->second.online == online) {
      return;
    }
    existing->second.online = online;
    existing->second.state_changed_at = now;
    if (online) {
      ++existing->second.online_transitions;
    }
  }

  mutable std::mutex presence_mutex_;
  std::map<std::string, PresenceRecord> presence_;
};

namespace {

class ZenohPublisher final : public TransportPublisher {
 public:
  explicit ZenohPublisher(zenoh::Publisher&& publisher)
      : publisher_(std::move(publisher)) {}

  bool Publish(const TransportEnvelope& envelope, std::string* error) override {
    std::vector<std::uint8_t> wire;
    if (!EncodeEnvelope(envelope, &wire, error)) {
      return false;
    }
    try {
      publisher_.put(zenoh::Bytes(std::move(wire)));
      return true;
    } catch (const std::exception& exception) {
      SetError(exception.what(), error);
      return false;
    }
  }

 private:
  zenoh::Publisher publisher_;
};

class ZenohSubscription final : public TransportSubscription {
 public:
  explicit ZenohSubscription(zenoh::Subscriber<void>&& subscriber)
      : subscriber_(std::move(subscriber)) {}

 private:
  zenoh::Subscriber<void> subscriber_;
};

class ZenohQueryable final : public TransportQueryable {
 public:
  explicit ZenohQueryable(zenoh::Queryable<void>&& queryable)
      : queryable_(std::move(queryable)) {}

 private:
  zenoh::Queryable<void> queryable_;
};

}  // namespace

ZenohTransport::ZenohTransport(const ZenohTransportConfig& config, std::string hostname)
    : impl_(std::make_shared<Impl>(config, hostname)) {}

ZenohTransport::~ZenohTransport() { Close(); }

std::shared_ptr<TransportPublisher> ZenohTransport::DeclarePublisher(
    const std::string& key, QosClass qos_class) {
  zenoh::Session::PublisherOptions options;
  const QosPolicy policy = GetQosPolicy(qos_class);
  options.congestion_control = policy.block_on_congestion
                                   ? Z_CONGESTION_CONTROL_BLOCK
                                   : Z_CONGESTION_CONTROL_DROP;
  options.priority = ToZenohPriority(qos_class);
  options.is_express = policy.express;
#if defined(Z_FEATURE_UNSTABLE_API)
  options.reliability = policy.reliable ? Z_RELIABILITY_RELIABLE
                                        : Z_RELIABILITY_BEST_EFFORT;
#endif
  auto publisher = impl_->Session().declare_publisher(
      zenoh::KeyExpr(key), std::move(options));
  return std::make_shared<ZenohPublisher>(std::move(publisher));
}

std::shared_ptr<TransportSubscription> ZenohTransport::Subscribe(
    const std::string& key_expression, EnvelopeCallback callback) {
  std::weak_ptr<Impl> weak_impl = impl_;
  auto on_sample = [weak_impl, callback = std::move(callback)](
                       const zenoh::Sample& sample) mutable {
    const std::shared_ptr<Impl> impl = weak_impl.lock();
    if (impl == nullptr) {
      return;
    }
    std::vector<std::uint8_t> bytes = sample.get_payload().as_vector();
    TransportEnvelope envelope;
    if (!DecodeEnvelope(bytes.data(), bytes.size(), &envelope)) {
      impl->receive_decode_errors_.fetch_add(1U);
      return;
    }
    try {
      callback(std::move(envelope));
    } catch (...) {
      impl->callback_drops_.fetch_add(1U);
    }
  };
  auto subscriber = impl_->Session().declare_subscriber(
      zenoh::KeyExpr(key_expression), std::move(on_sample), zenoh::closures::none);
  return std::make_shared<ZenohSubscription>(std::move(subscriber));
}

std::shared_ptr<TransportQueryable> ZenohTransport::DeclareQueryable(
    const std::string& key, QueryHandler handler) {
  std::weak_ptr<Impl> weak_impl = impl_;
  auto on_query = [weak_impl, handler = std::move(handler), key](
                      const zenoh::Query& query) mutable {
    const std::shared_ptr<Impl> impl = weak_impl.lock();
    if (impl == nullptr) {
      return;
    }

    const auto query_payload = query.get_payload();
    if (!query_payload.has_value()) {
      query.reply_err(zenoh::Bytes("missing NetBridge request envelope"));
      return;
    }
    std::vector<std::uint8_t> bytes = query_payload->get().as_vector();
    TransportEnvelope request;
    std::string decode_error;
    if (!DecodeEnvelope(bytes.data(), bytes.size(), &request, &decode_error)) {
      impl->receive_decode_errors_.fetch_add(1U);
      query.reply_err(zenoh::Bytes(std::move(decode_error)));
      return;
    }

    auto saved_query = std::make_shared<zenoh::Query>(query.clone());
    const bool accepted = impl->executor_.TrySubmit(
        [saved_query, handler, key, request = std::move(request)]() mutable {
          TransportEnvelope response;
          std::string error;
          try {
            if (!handler(request, &response, &error)) {
              if (error.empty()) {
                error = "service handler rejected request";
              }
              saved_query->reply_err(zenoh::Bytes(std::move(error)));
              return;
            }
            std::vector<std::uint8_t> wire;
            if (!EncodeEnvelope(response, &wire, &error)) {
              saved_query->reply_err(zenoh::Bytes(std::move(error)));
              return;
            }
            zenoh::Query::ReplyOptions options;
            options.is_express = true;
            saved_query->reply(zenoh::KeyExpr(key), zenoh::Bytes(std::move(wire)),
                               std::move(options));
          } catch (const std::exception& exception) {
            saved_query->reply_err(zenoh::Bytes(exception.what()));
          }
        });
    if (!accepted) {
      impl->service_queue_drops_.fetch_add(1U);
      query.reply_err(zenoh::Bytes("NetBridge service worker queue is full"));
    }
  };

  zenoh::Session::QueryableOptions options;
  options.complete = true;
  auto queryable = impl_->Session().declare_queryable(
      zenoh::KeyExpr(key), std::move(on_query), zenoh::closures::none,
      std::move(options));
  return std::make_shared<ZenohQueryable>(std::move(queryable));
}

bool ZenohTransport::Query(const std::string& key,
                           const TransportEnvelope& request,
                           std::uint64_t timeout_ms,
                           TransportEnvelope* response,
                           std::string* error) {
  if (response == nullptr) {
    SetError("response is null", error);
    return false;
  }
  std::vector<std::uint8_t> wire;
  if (!EncodeEnvelope(request, &wire, error)) {
    return false;
  }

  struct QueryState {
    std::mutex mutex;
    std::condition_variable condition;
    bool done{false};
    bool received{false};
    TransportEnvelope response;
    std::string error;
  };
  auto state = std::make_shared<QueryState>();
  auto on_reply = [state](const zenoh::Reply& reply) {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (state->received) {
      return;
    }
    if (!reply.is_ok()) {
      state->error = reply.get_err().get_payload().as_string();
      state->received = true;
      state->condition.notify_all();
      return;
    }
    const std::vector<std::uint8_t> bytes = reply.get_ok().get_payload().as_vector();
    if (!DecodeEnvelope(bytes.data(), bytes.size(), &state->response, &state->error)) {
      state->received = true;
      state->condition.notify_all();
      return;
    }
    state->received = true;
    state->condition.notify_all();
  };
  auto on_done = [state]() {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->done = true;
    state->condition.notify_all();
  };

  try {
    zenoh::Session::GetOptions options;
    options.target = zenoh::QueryTarget::Z_QUERY_TARGET_BEST_MATCHING;
    options.priority = Z_PRIORITY_INTERACTIVE_HIGH;
    options.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
    options.is_express = true;
    options.timeout_ms = timeout_ms;
    options.payload = zenoh::Bytes(std::move(wire));
    impl_->Session().get(zenoh::KeyExpr(key), "", std::move(on_reply),
                         std::move(on_done), std::move(options));
  } catch (const std::exception& exception) {
    SetError(exception.what(), error);
    return false;
  }

  std::unique_lock<std::mutex> lock(state->mutex);
  const auto local_timeout = std::chrono::milliseconds(timeout_ms + 250U);
  state->condition.wait_for(lock, local_timeout,
                            [state]() { return state->received || state->done; });
  if (!state->received) {
    impl_->service_timeouts_.fetch_add(1U);
    SetError("Zenoh service query timed out", error);
    return false;
  }
  if (!state->error.empty()) {
    if (IsTimeoutError(state->error)) {
      impl_->service_timeouts_.fetch_add(1U);
      SetError("Zenoh service query timed out", error);
      return false;
    }
    SetError(state->error, error);
    return false;
  }
  *response = std::move(state->response);
  return true;
}

TransportStats ZenohTransport::GetStats() const { return impl_->Stats(); }

std::vector<NodePresence> ZenohTransport::GetNodePresence() const {
  return impl_->Presence();
}

void ZenohTransport::Close() {
  if (impl_ != nullptr) {
    impl_->Close();
  }
}

}  // namespace transport
}  // namespace swarm_ros_bridge
