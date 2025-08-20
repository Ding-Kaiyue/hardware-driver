#ifndef __EVENT_BUS_HPP__
#define __EVENT_BUS_HPP__

#include <functional>
#include <unordered_map>
#include <vector>
#include <memory>
#include <mutex>
#include <typeinfo>
#include <typeindex>
#include <string>
#include <any>
#include <atomic>
#include <algorithm>

namespace hardware_driver {
namespace event {

// 通用事件基类
class Event {
public:
    virtual ~Event() = default;
    
    // 获取事件类型名称
    virtual std::string get_type_name() const = 0;
    
    // 获取事件主题（可选，用于主题过滤）
    virtual std::string get_topic() const { return ""; }
};

// 事件处理器基类
class EventHandler {
public:
    virtual ~EventHandler() = default;
    virtual void handle_event(const std::shared_ptr<Event>& event) = 0;
};

// 类型安全的事件处理器模板
template<typename EventType>
class TypedEventHandler : public EventHandler {
public:
    using HandlerFunction = std::function<void(const std::shared_ptr<EventType>&)>;
    
    explicit TypedEventHandler(HandlerFunction handler) : handler_(handler) {}
    
    void handle_event(const std::shared_ptr<Event>& event) override {
        if (auto typed_event = std::dynamic_pointer_cast<EventType>(event)) {
            handler_(typed_event);
        }
    }
    
private:
    HandlerFunction handler_;
};

// 通用事件总线
class EventBus {
public:
    EventBus() = default;
    ~EventBus() = default;
    
    // 禁止拷贝和移动
    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;
    EventBus(EventBus&&) = delete;
    EventBus& operator=(EventBus&&) = delete;
    
    // 订阅指定类型的事件 - 返回处理器的强引用以便用户管理生命周期
    template<typename EventType>
    std::shared_ptr<EventHandler> subscribe(std::function<void(const std::shared_ptr<EventType>&)> handler) {
        std::lock_guard<std::mutex> lock(handlers_mutex_);
        std::type_index type_idx(typeid(EventType));
        
        auto typed_handler = std::make_shared<TypedEventHandler<EventType>>(handler);
        handlers_[type_idx].push_back(typed_handler);
        return typed_handler;
    }
    
    // 订阅指定主题的事件
    template<typename EventType>
    std::shared_ptr<EventHandler> subscribe_topic(const std::string& topic, 
                        std::function<void(const std::shared_ptr<EventType>&)> handler) {
        std::lock_guard<std::mutex> lock(topic_handlers_mutex_);
        
        auto typed_handler = std::make_shared<TypedEventHandler<EventType>>(handler);
        topic_handlers_[topic].push_back(typed_handler);
        return typed_handler;
    }
    
    // 取消订阅
    template<typename EventType>
    void unsubscribe(std::shared_ptr<EventHandler> handler) {
        std::lock_guard<std::mutex> lock(handlers_mutex_);
        std::type_index type_idx(typeid(EventType));
        
        auto it = handlers_.find(type_idx);
        if (it != handlers_.end()) {
            it->second.erase(
                std::remove_if(it->second.begin(), it->second.end(),
                    [&handler](const std::weak_ptr<EventHandler>& weak_handler) {
                        return weak_handler.lock() == handler;
                    }),
                it->second.end());
        }
    }
    
    // 发布事件
    void publish(std::shared_ptr<Event> event) {
        if (!event) return;
        
        // 按类型分发
        publish_by_type(event);
        
        // 按主题分发
        std::string topic = event->get_topic();
        if (!topic.empty()) {
            publish_by_topic(topic, event);
        }
    }
    
    // 创建并发布事件的便捷方法
    template<typename EventType, typename... Args>
    void emit(Args&&... args) {
        auto event = std::make_shared<EventType>(std::forward<Args>(args)...);
        publish(event);
    }
    
    // 获取统计信息
    struct Statistics {
        size_t total_handlers = 0;
        size_t total_topic_handlers = 0;
        size_t events_published = 0;
    };
    
    Statistics get_statistics() const {
        Statistics stats;
        
        // 分别锁定避免死锁
        {
            std::lock_guard<std::mutex> lock(handlers_mutex_);
            for (const auto& [type, handler_list] : handlers_) {
                stats.total_handlers += handler_list.size();
            }
        }
        
        {
            std::lock_guard<std::mutex> lock(topic_handlers_mutex_);
            for (const auto& [topic, handler_list] : topic_handlers_) {
                stats.total_topic_handlers += handler_list.size();
            }
        }
        
        stats.events_published = events_published_;
        return stats;
    }
    
    // 清理失效的处理器
    void cleanup_handlers() {
        // 分别清理避免死锁
        {
            std::lock_guard<std::mutex> lock(handlers_mutex_);
            for (auto& [type, handler_list] : handlers_) {
                handler_list.erase(
                    std::remove_if(handler_list.begin(), handler_list.end(),
                        [](const std::weak_ptr<EventHandler>& weak_handler) {
                            return weak_handler.expired();
                        }),
                    handler_list.end());
            }
        }
        
        {
            std::lock_guard<std::mutex> lock(topic_handlers_mutex_);
            for (auto& [topic, handler_list] : topic_handlers_) {
                handler_list.erase(
                    std::remove_if(handler_list.begin(), handler_list.end(),
                        [](const std::weak_ptr<EventHandler>& weak_handler) {
                            return weak_handler.expired();
                        }),
                    handler_list.end());
            }
        }
    }
    
private:
    void publish_by_type(const std::shared_ptr<Event>& event) {
        std::lock_guard<std::mutex> lock(handlers_mutex_);
        std::type_index type_idx(typeid(*event));
        
        auto it = handlers_.find(type_idx);
        if (it != handlers_.end()) {
            for (auto& weak_handler : it->second) {
                if (auto handler = weak_handler.lock()) {
                    try {
                        handler->handle_event(event);
                    } catch (const std::exception& e) {
                        // 记录错误但不中断其他处理器
                        // TODO: 添加日志系统
                    }
                }
            }
        }
        events_published_++;
    }
    
    void publish_by_topic(const std::string& topic, const std::shared_ptr<Event>& event) {
        std::lock_guard<std::mutex> lock(topic_handlers_mutex_);
        
        auto it = topic_handlers_.find(topic);
        if (it != topic_handlers_.end()) {
            for (auto& weak_handler : it->second) {
                if (auto handler = weak_handler.lock()) {
                    try {
                        handler->handle_event(event);
                    } catch (const std::exception& e) {
                        // 记录错误但不中断其他处理器
                    }
                }
            }
        }
    }
    
    // 按类型索引存储处理器
    std::unordered_map<std::type_index, std::vector<std::weak_ptr<EventHandler>>> handlers_;
    mutable std::mutex handlers_mutex_;
    
    // 按主题存储处理器
    std::unordered_map<std::string, std::vector<std::weak_ptr<EventHandler>>> topic_handlers_;
    mutable std::mutex topic_handlers_mutex_;
    
    // 统计信息
    mutable std::atomic<size_t> events_published_{0};
};

// 全局事件总线单例（可选使用）
class GlobalEventBus {
public:
    static EventBus& instance() {
        static EventBus bus;
        return bus;
    }
    
private:
    GlobalEventBus() = default;
};

}  // namespace event
}  // namespace hardware_driver

#endif  // __EVENT_BUS_HPP__