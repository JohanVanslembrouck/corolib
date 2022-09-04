/**
 *  Filename: p0320.cpp
 *  Description: Illustrates use of co_yield to define a recursive generator type.
 *  The generator is accompanied by an iterator.
 *  The generator and iterator are independent of the applications in which they are used.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com)
 */

#include <functional>
#include <cassert>
#include <atomic>
#include <thread>

#include <stdarg.h>

//--------------------------------------------------------------

const int PRI1 = 0x01;
const int PRI2 = 0x02;
const int PRI3 = 0x04;
const int PRI4 = 0x08;

uint64_t threadids[128];

int get_thread_number(uint64_t id)
{
    for (int i = 0; i < 128; i++)
    {
        if (threadids[i] == id)
            return i;
        if (threadids[i] == 0) {
            threadids[i] = id;
            return i;
        }
    }
    return -1;
}

int get_thread_number64(uint64_t id)
{
    for (int i = 0; i < 128; i++)
    {
        if (threadids[i] == id)
            return i;
        if (threadids[i] == 0) {
            threadids[i] = id;
            return i;
        }
    }
    return -1;
}

int get_thread_number32(uint32_t id)
{
    for (int i = 0; i < 128; i++)
    {
        if (threadids[i] == id)
            return i;
        if (threadids[i] == 0) {
            threadids[i] = id;
            return i;
        }
    }
    return -1;
}

uint64_t get_thread_id()
{
    auto id = std::this_thread::get_id();
    uint64_t* ptr = (uint64_t*)&id;
    return (uint64_t)(*ptr);
}

const int priority = 0x01;

void print(int pri, const char* fmt, ...)
{
    va_list arg;
    char msg[256];

    va_start(arg, fmt);
    int n = vsprintf_s(msg, fmt, arg);
    va_end(arg);

    int threadid = (sizeof(std::thread::id) == sizeof(uint32_t)) ?
        get_thread_number32((uint32_t)get_thread_id()) :
        get_thread_number64(get_thread_id());
    if (priority & pri)
        fprintf(stderr, "%02d: %s", threadid, msg);
}

//--------------------------------------------------------------
//--------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// Copyright (c) Lewis Baker
// Licenced under MIT license. See LICENSE.txt for details.
///////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <cassert>
#include <experimental/coroutine>

template<typename T>
class recursive_generator
{
public:

    class promise_type final
    {
    public:

        promise_type() noexcept
            : m_value(nullptr)
            , m_exception(nullptr)
            , m_root(this)
            , m_parentOrLeaf(this)
        {
            print(PRI2, "recursive_generator::promise_type::promise_type()\n");
        }

        promise_type(const promise_type&) = delete;
        promise_type(promise_type&&) = delete;

        auto get_return_object() noexcept
        {
            print(PRI2, "recursive_generator::promise_type::get_return_object()\n");
            return recursive_generator<T>{ *this };
        }

        std::experimental::suspend_always initial_suspend() noexcept
        {
            print(PRI2, "recursive_generator::promise_type::initial_suspend()\n");
            return {};
        }

        std::experimental::suspend_always final_suspend() noexcept
        {
            print(PRI2, "recursive_generator::promise_type::final_suspend()\n");
            return {};
        }

        void unhandled_exception() noexcept
        {
            print(PRI2, "recursive_generator::promise_type::unhandled_exception()\n");
            m_exception = std::current_exception();
        }

        void return_void() noexcept 
        {
            print(PRI2, "return_void()\n");
        }

        std::experimental::suspend_always yield_value(T& value) noexcept
        {
            print(PRI2, "recursive_generator::promise_type::yield_value(T& value)\n");
            m_value = std::addressof(value);
            return {};
        }

        std::experimental::suspend_always yield_value(T&& value) noexcept
        {
            print(PRI2, "recursive_generator::promise_type::yield_value(T&& value)\n");
            m_value = std::addressof(value);
            return {};
        }

        auto yield_value(recursive_generator&& generator) noexcept
        {
            print(PRI2, "recursive_generator::promise_type::yield_value(recursive_generator&& generator)\n");
            return yield_value(generator);
        }

        auto yield_value(recursive_generator& generator) noexcept
        {
            print(PRI2, "recursive_generator::promise_type::yield_value(recursive_generator& generator)\n");

            struct awaitable
            {
                awaitable(promise_type* childPromise)
                    : m_childPromise(childPromise)
                {
                    print(PRI2, "awaitable::awaitable(promise_type* childPromise)\n");
                }

                bool await_ready() noexcept
                {
                    print(PRI2, "awaitable::await_ready()\n");
                    return this->m_childPromise == nullptr;
                }

                void await_suspend(std::experimental::coroutine_handle<promise_type>) noexcept
                {
                    print(PRI2, "awaitable::await_suspend(std::experimental::coroutine_handle<promise_type>)\n");
                }

                void await_resume()
                {
                    print(PRI2, "awaitable::await_resume()\n");
                    if (this->m_childPromise != nullptr)
                    {
                        this->m_childPromise->throw_if_exception();
                    }
                }

            private:
                promise_type* m_childPromise;
            };

            if (generator.m_promise != nullptr)
            {
                m_root->m_parentOrLeaf = generator.m_promise;
                generator.m_promise->m_root = m_root;
                generator.m_promise->m_parentOrLeaf = this;
                generator.m_promise->resume();

                if (!generator.m_promise->is_complete())
                {
                    return awaitable{ generator.m_promise };
                }

                m_root->m_parentOrLeaf = this;
            }

            return awaitable{ nullptr };
        }

        // Don't allow any use of 'co_await' inside the recursive_generator coroutine.
        template<typename U>
        std::experimental::suspend_never await_transform(U&& value) = delete;

        void destroy() noexcept
        {
            print(PRI2, "recursive_generator::promise_type::destroy()\n");
            std::experimental::coroutine_handle<promise_type>::from_promise(*this).destroy();
        }

        void throw_if_exception()
        {
            print(PRI2, "recursive_generator::promise_type::throw_if_exception()\n");
            if (m_exception != nullptr)
            {
                std::rethrow_exception(std::move(m_exception));
            }
        }

        bool is_complete() noexcept
        {
            print(PRI2, "recursive_generator::promise_type::is_complete()\n");
            return std::experimental::coroutine_handle<promise_type>::from_promise(*this).done();
        }

        T& value() noexcept
        {
            print(PRI2, "recursive_generator::promise_type::value()\n");
            assert(this == m_root);
            assert(!is_complete());
            return *(m_parentOrLeaf->m_value);
        }

        void pull() noexcept
        {
            print(PRI2, "recursive_generator::promise_type::pull()\n");
            assert(this == m_root);
            assert(!m_parentOrLeaf->is_complete());

            m_parentOrLeaf->resume();

            while (m_parentOrLeaf != this && m_parentOrLeaf->is_complete())
            {
                m_parentOrLeaf = m_parentOrLeaf->m_parentOrLeaf;
                m_parentOrLeaf->resume();
            }
        }

    private:

        void resume() noexcept
        {
            print(PRI2, "recursive_generator::promise_type::resume()\n");
            std::experimental::coroutine_handle<promise_type>::from_promise(*this).resume();
        }

        std::add_pointer_t<T> m_value;
        std::exception_ptr m_exception;

        promise_type* m_root;

        // If this is the promise of the root generator then this field
        // is a pointer to the leaf promise.
        // For non-root generators this is a pointer to the parent promise.
        promise_type* m_parentOrLeaf;

    };

    recursive_generator() noexcept
        : m_promise(nullptr)
    {
        print(PRI2, "recursive_generator::recursive_generator()\n");
    }

    recursive_generator(promise_type& promise) noexcept
        : m_promise(&promise)
    {
        print(PRI2, "recursive_generator::recursive_generator(promise_type& promise)\n");
    }

    recursive_generator(recursive_generator&& other) noexcept
        : m_promise(other.m_promise)
    {
        print(PRI2, "recursive_generator::recursive_generator(recursive_generator&& other)\n");
        other.m_promise = nullptr;
    }

    recursive_generator(const recursive_generator& other) = delete;
    recursive_generator& operator=(const recursive_generator& other) = delete;

    ~recursive_generator()
    {
        print(PRI2, "recursive_generator::~recursive_generator()\n");
        if (m_promise != nullptr)
        {
            m_promise->destroy();
        }
    }

    recursive_generator& operator=(recursive_generator&& other) noexcept
    {
        print(PRI2, "recursive_generator::operator=(recursive_generator&& other)\n");
        if (this != &other)
        {
            if (m_promise != nullptr)
            {
                m_promise->destroy();
            }

            m_promise = other.m_promise;
            other.m_promise = nullptr;
        }

        return *this;
    }

    class iterator
    {
    public:

        using iterator_category = std::input_iterator_tag;
        // What type should we use for counting elements of a potentially infinite sequence?
        using difference_type = std::ptrdiff_t;
        using value_type = std::remove_reference_t<T>;
        using reference = std::conditional_t<std::is_reference_v<T>, T, T&>;
        using pointer = std::add_pointer_t<T>;

        iterator() noexcept
            : m_promise(nullptr)
        {
            print(PRI2, "recursive_generator::iterator::iterator()\n");
        }

        explicit iterator(promise_type* promise) noexcept
            : m_promise(promise)
        {
            print(PRI2, "recursive_generator::iterator::iterator(promise_type* promise)\n");
        }

        bool operator==(const iterator& other) const noexcept
        {
            print(PRI2, "recursive_generator::iterator::operator==(const iterator& other)\n");
            return m_promise == other.m_promise;
        }

        bool operator!=(const iterator& other) const noexcept
        {
            print(PRI2, "recursive_generator::iterator::operator!=(const iterator& other)\n");
            return m_promise != other.m_promise;
        }

        iterator& operator++()
        {
            print(PRI2, "recursive_generator::iterator::operator++()\n");

            assert(m_promise != nullptr);
            assert(!m_promise->is_complete());

            m_promise->pull();
            if (m_promise->is_complete())
            {
                auto* temp = m_promise;
                m_promise = nullptr;
                temp->throw_if_exception();
            }

            return *this;
        }

        void operator++(int)
        {
            print(PRI2, "recursive_generator::iterator::operator++(int)\n");
            (void)operator++();
        }

        reference operator*() const noexcept
        {
            print(PRI2, "recursive_generator::iterator::operator*()\n");
            assert(m_promise != nullptr);
            return static_cast<reference>(m_promise->value());
        }

        pointer operator->() const noexcept
        {
            print(PRI2, "recursive_generator::iterator::operator->()\n");
            return std::addressof(operator*());
        }

    private:

        promise_type* m_promise;

    };

    iterator begin()
    {
        print(PRI2, "recursive_generator::begin()\n");
        if (m_promise != nullptr)
        {
            m_promise->pull();
            if (!m_promise->is_complete())
            {
                return iterator(m_promise);
            }

            m_promise->throw_if_exception();
        }

        return iterator(nullptr);
    }

    iterator end() noexcept
    {
        print(PRI2, "recursive_generator::end()\n");
        return iterator(nullptr);
    }

    void swap(recursive_generator& other) noexcept
    {
        print(PRI2, "recursive_generator::swap(recursive_generator& other)\n");
        std::swap(m_promise, other.m_promise);
    }

private:

    friend class promise_type;

    promise_type* m_promise;

};

//--------------------------------------------------------------
//--------------------------------------------------------------

recursive_generator<std::uint32_t> iterate_range(std::uint32_t begin, std::uint32_t end)
{
    print(PRI2, "iterate_range(std::uint32_t begin, std::uint32_t end)\n");
    if ((end - begin) <= 10u)
    {
        for (std::uint32_t i = begin; i < end; ++i)
        {
            co_yield i;
        }
    }
    else
    {
        std::uint32_t mid = begin + (end - begin) / 2;
        co_yield iterate_range(begin, mid);
        co_yield iterate_range(mid, end);
    }
}

recursive_generator<int> range(int start, int end)
{
    print(PRI2, "range(int start, int end)\n");
    while (start < end)
    {
        co_yield start++;
    }
}

recursive_generator<int> range_chunks(int start, int end, int runLength, int stride)
{
    print(PRI2, "range_chunks(int start, int end, int runLength, int stride)\n");
    while (start < end)
    {
        co_yield range(start, std::min(end, start + runLength));
        start += stride;
    }
}

int main()
{
    print(PRI2, "main()\n");

    const std::uint32_t count = 1000;        // 100000

    std::uint64_t sum = 0;
    for (auto i : iterate_range(0, count))
    {
        sum += i;
    }

    {
        auto a = iterate_range(5, 30);
        auto b = iterate_range(5, 30);
        //CHECK(std::equal(a.begin(), a.end(), b.begin(), b.end()));
    }

    {
        auto a = iterate_range(5, 30);
        auto b = iterate_range(5, 300);
        //CHECK(!std::equal(a.begin(), a.end(), b.begin(), b.end()));
    }

    recursive_generator<int> gen = range_chunks(0, 30, 5, 10);

    auto it = gen.begin();
    print(PRI1, "main(): *it = %d\n", *it);
    for (int i = 0; i < 14; i++)
        print(PRI1, "main(): *++it = %d\n", *++it);
    if (++it == gen.end())
        print(PRI1, "main(): ++it == gen.end()\n");

    return 0;
}