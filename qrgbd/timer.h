#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>

namespace ns_timer
{
    namespace DurationType
    {
        struct NS : public std::chrono::nanoseconds
        {
            using chrono_type = std::chrono::nanoseconds;
            using chrono_type::chrono_type;
            static std::string unit_str()
            {
                return "NS";
            }
        };

        struct US : public std::chrono::microseconds
        {
            using chrono_type = std::chrono::microseconds;
            using chrono_type::chrono_type;
            static std::string unit_str()
            {
                return "US";
            }
        };

        struct MS : public std::chrono::milliseconds
        {
            using chrono_type = std::chrono::milliseconds;
            using chrono_type::chrono_type;
            static std::string unit_str()
            {
                return "MS";
            }
        };

        struct S : public std::chrono::seconds
        {
            using chrono_type = std::chrono::seconds;
            using chrono_type::chrono_type;
            static std::string unit_str()
            {
                return "S";
            }
        };

        struct MIN : public std::chrono::minutes
        {
            using chrono_type = std::chrono::minutes;
            using chrono_type::chrono_type;
            static std::string unit_str()
            {
                return "MIN";
            }
        };

        struct H : public std::chrono::hours
        {
            using chrono_type = std::chrono::hours;
            using chrono_type::chrono_type;
            static std::string unit_str()
            {
                return "H";
            }
        };

    } // namespace DurationType

    /**
   * @brief sleep for the 'period'
   *
   * @tparam DurationType the type of duration
   * @param period the time to sleep
   */
    template <typename DurationType = DurationType::MS>
    void sleep(const typename DurationType::rep &period)
    {
        std::this_thread::sleep_for(DurationType(period));
        return;
    }

    /**
   * @brief the Timer class to timing
   *
   * @tparam ClockType the type of the clock used, eg:std::chrono::system_clock
   */
    template <typename ClockType = std::chrono::system_clock>
    class Timer
    {
    public:
        using default_dur_type = DurationType::MS;
        using clock_type = ClockType;
        using time_point_type = typename clock_type::time_point;

    private:
        time_point_type _start;
        time_point_type _last;

    public:
        Timer() : _start(clock_type::now()), _last(clock_type::now()) {}

        /**
     * @brief get the last duration from the 'start' time point to 'now' time point
     *
     * @tparam DurationType the type of std::duration, eg: std::chrono::milliseconds, std::chrono::seconds
     * @return float the duration count
     */
        template <typename DurationType = default_dur_type>
        float last_elapsed()
        {
            return this->count<DurationType>(this->_last);
        }

        /**
     * @brief get the total duration from the 'start' time point to 'now' time point
     *
     * @tparam DurationType the type of std::duration, eg: std::chrono::milliseconds, std::chrono::seconds
     * @return float the duration count
     */
        template <typename DurationType = default_dur_type>
        float total_elapsed()
        {
            return this->count<DurationType>(this->_start);
        }

        /**
     * @brief get the last duration from the 'start' time point to 'now' time
     * point
     *
     * @tparam DurationType the type of std::duration, eg: std::chrono::milliseconds, std::chrono::seconds
     * @param desc the describe of this duration
     * @param prec the precision for float value
     * @return std::string the duration string
     */
        template <typename DurationType = default_dur_type>
        std::string last_elapsed(const std::string &desc, std::size_t prec = 5)
        {
            std::stringstream stream;
            stream << std::fixed << std::setprecision(prec);
            stream << "{'";
            stream << desc << "': " << this->last_elapsed<DurationType>();
            stream << '(' << DurationType::unit_str() << ')';
            stream << '}';
            return stream.str();
        }

        /**
     * @brief get the total duration from the 'start' time point to 'now' time point
     *
     * @tparam DurationType the type of std::duration, eg: std::chrono::milliseconds, std::chrono::seconds
     * @param desc the describe of this duration
     * @param prec the precision for float value
     * @return std::string the duration string
     */
        template <typename DurationType = default_dur_type>
        std::string total_elapsed(const std::string &desc, std::size_t prec = 5)
        {
            std::stringstream stream;
            stream << std::fixed << std::setprecision(prec);
            stream << "{'";
            stream << desc << "': " << this->total_elapsed<DurationType>();
            stream << '(' << DurationType::unit_str() << ')';
            stream << '}';
            return stream.str();
        }

        /**
     * @brief sleep for the 'period'
     *
     * @tparam DurationType the type of duration
     * @param period the time to sleep
     */
        template <typename DurationType = default_dur_type>
        void sleep(const typename DurationType::rep &period)
        {
            std::this_thread::sleep_for(DurationType(period));
            return;
        }

        /**
     * @brief restart the timer
     */
        void reStart()
        {
            this->_last = clock_type::now();
            return;
        }

        /**
     * @brief reboot the timer
     */
        void reBoot()
        {
            this->_start = clock_type::now();
            return;
        }

    protected:
        /**
     * @brief Get the timer's count
     *
     * @tparam DurationType the type of std::duration
     * @param ref the reference duration to count. eg: this->_start, this->_last
     * @return float the count value
     */
        template <typename DurationType = default_dur_type>
        float count(const time_point_type &ref)
        {
            using cast_type =
                std::chrono::duration<float, typename DurationType::period>;
            auto dur = std::chrono::duration_cast<cast_type>(clock_type::now() - ref);
            this->_last = clock_type::now();
            return dur.count();
        }

    private:
        Timer(const Timer &) = delete;
        Timer(Timer &&) = delete;
        Timer &operator=(const Timer &) = delete;
        Timer &operator=(Timer &&) = delete;
    };

} // namespace ns_timer
