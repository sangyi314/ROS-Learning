#pragma once 

#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>

class MovingAverage
{
    public:
        explicit MovingAverage(std::size_t window_size)
        :  window_size_ (window_size)
        {
            if (window_size_ == 0)
            {
                throw std::invalid_argument("window_size must be positive");
            }
        }

        double push(double value)
        {
            if(!std::isfinite(value))
            {
                throw std::invalid_argument("value must be finite");
            }
            values_.push_back(value);
            if (values_.size() > window_size_)
            {
                values_.pop_front();
            }
            double total{0.0};
            for (double i : values_)
            {
                total += i;
            }
            return total/static_cast<double>(values_.size());
        }

        void clear()
        {
            values_.clear();
        }

        std::size_t size()
        {
            return values_.size();
        }
    private:
    std::size_t window_size_;
    std::deque<double> values_;
};
