#pragma once

#include <algorithm>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

/**
 * @brief The MessageTraits struct is used to specialize timestamp and frame retrieval for different
 * data types.
 *
 * @tparam T The data type for which timestamp and frame retrieval is specialized.
 */
template <typename T>
struct MessageTraits
{
  /**
   * @brief GetTimestamp retrieves the timestamp for the specified data.
   *
   * @param data The data for which the timestamp is retrieved.
   * @return The timestamp.
   */
  static std::chrono::system_clock::time_point getTimestamp(const T & data)
  {
    return std::chrono::system_clock::time_point(
      std::chrono::nanoseconds(data->header.stamp.nanosec) +
      std::chrono::seconds(data->header.stamp.sec));
  }

  /**
   * @brief GetFrame retrieves the frame for the specified data.
   *
   * @param data The data for which the frame is retrieved.
   * @return The frame.
   */
  static std::string getFrame(const T & data) { return data->header.frame_id; }
};

/**
 * @brief A class representing a message buffer that stores messages with timestamps.
 *
 * This class allows you to store and retrieve messages with timestamps. You can add messages
 * to the buffer using addData and retrieve messages based on timestamps using getDataByTimestamp.
 *
 * @tparam T The type of data to be stored in the messages.
 */
template <typename T>
class MessageBuffer
{
private:
  /**
   * @brief A struct representing a message with a timestamp.
   */
  struct Message
  {
    std::chrono::system_clock::time_point timestamp; /**< The timestamp of the message. */
    std::string frame;                               /**< A frame identifier for the message. */
    T data;                                          /**< The data stored in the message. */
  };

  std::deque<Message> buffer_; /**< The deque to store messages. */
  std::mutex mutex_;           /**< Mutex for thread safety. */
  size_t buffer_max_size_ = 1; /**< The maximum size of the buffer. */

public:
  /**
   * @brief Add data to the message buffer.
   *
   * This method adds data to the message buffer. If the buffer size exceeds the maximum size,
   * the oldest data will be removed.
   *
   * @param data The data to be added to the buffer.
   */
  void addData(const T & data)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Get timestamp and frame using template specialization
    auto timestamp = MessageTraits<T>::getTimestamp(data);
    auto frame = MessageTraits<T>::getFrame(data);

    // Add the message to the buffer
    buffer_.push_back({timestamp, frame, data});

    // If the buffer size exceeds the maximum, remove the oldest message
    if (buffer_.size() > buffer_max_size_) {
      buffer_.pop_front();
    }
  }

  /**
   * @brief Retrieve a message from the buffer based on the specified timestamp and policy
   * parameters.
   *
   * This method allows you to retrieve a message from the buffer based on the provided timestamp
   * and policy parameters.
   *
   * @param timestamp The timestamp to match when retrieving a message.
   * @param exact_match If true, find a message with the exact same timestamp.
   *                    If false, find the closest message to the specified timestamp.
   * @param timeout If timeout is greater than 0, it finds the closest message to the specified
   * timestamp within the given timeout period. If timeout is 0 or less, it finds the closest
   * message to the specified timestamp without considering any timeout.
   * @return An optional containing the retrieved message if found, or std::nullopt if no matching
   * message is found.
   */
  std::optional<Message> getDataByTimestamp(
    const std::chrono::system_clock::time_point & timestamp, bool exact_match = true,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (exact_match) {
      // Find a message with the exact same timestamp.
      auto it = std::find_if(buffer_.begin(), buffer_.end(), [&](const Message & msg) {
        return msg.timestamp == timestamp;
      });

      if (it != buffer_.end()) {
        return *it;
      }
    } else {
      // Find the closest message to the specified timestamp.
      auto closest = buffer_.end();

      for (auto it = buffer_.begin(); it != buffer_.end(); ++it) {
        if (
          closest == buffer_.end() || std::abs((it->timestamp - timestamp).count()) <
                                        std::abs((closest->timestamp - timestamp).count())) {
          closest = it;
        }
      }

      if (
        closest != buffer_.end() &&
        (timeout.count() == 0 ||
         std::abs((closest->timestamp - timestamp).count()) <= timeout.count())) {
        return *closest;
      }
    }

    // No matching message found, return std::nullopt.
    return std::nullopt;
  }
};
