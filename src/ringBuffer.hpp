#ifndef __RINGBUFFER__
#define __RINGBUFFER__

#include <vector>

using std::vector;

template <typename T>
class ringBuffer
{
protected:
    vector<T> buffer;
    size_t head{0};
    size_t tail{0};
    size_t len{0};

public:
    explicit ringBuffer(size_t num) : len(num + 1)
    {
        buffer.resize(num + 1);
    }

    void push_back(T &&data)
    {
        buffer[tail++] = data;
        tail %= len;

        if (tail == head)
        { // ringBuffer is full
            ++head;
            head %= len;
        }
    }

    T &last()
    {
        T &ret = buffer[((tail - 1) + len) % len];

        return ret;
    }

    T &secondLast()
    {
        T &ret = buffer[((tail - 2) + len) % len];

        return ret;
    }

    int size(){
        return (tail - head + len) % len;
    }
};

#endif /* __RINGBUFFER__ */