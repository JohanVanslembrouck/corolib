/**
 *  Filename: p0340.cpp
 *  Description: Illustrates use of co_yield to define an asynchronous generator type.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *  Based upon: 
 */

#include <functional>
#include <cassert>
#include <atomic>
#include <iostream>
#include <thread>

#include <stdarg.h>

const int priority = 0x01;

#include "print.h"

//--------------------------------------------------------------
//--------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// Copyright (c) Lewis Baker
// Licenced under MIT license. See LICENSE.txt for details.
///////////////////////////////////////////////////////////////////////////////

#include <coroutine>

template<typename T>
class async_generator_iterator;
class async_generator_yield_operation;
class async_generator_advance_operation;

template<typename T>
class async_generator;

class async_generator_promise_base
{
public:
    async_generator_promise_base() noexcept
        : m_exception(nullptr)
    {
        print(PRI2, "async_generator_promise_base::async_generator_promise_base()\n");
        // Other variables left intentionally uninitialised as they're
        // only referenced in certain states by which time they should
        // have been initialised.
    }

    async_generator_promise_base(const async_generator_promise_base& other) = delete;
    async_generator_promise_base& operator=(const async_generator_promise_base& other) = delete;

    std::suspend_always initial_suspend() const noexcept
    {
        print(PRI2, "async_generator_promise_base::initial_suspend()\n");
        return {};
    }

    async_generator_yield_operation final_suspend() noexcept;

    void unhandled_exception() noexcept
    {
        print(PRI2, "async_generator_promise_base::unhandled_exception()\n");
        m_exception = std::current_exception();
    }

    void return_void() noexcept
    {
        print(PRI2, "async_generator_promise_base::return_void()\n");
    }

    /// Query if the generator has reached the end of the sequence.
    ///
    /// Only valid to call after resuming from an awaited advance operation.
    /// i.e. Either a begin() or iterator::operator++() operation.
    bool finished() const noexcept
    {
        print(PRI2, "async_generator_promise_base::finished()\n");
        return m_currentValue == nullptr;
    }

    void rethrow_if_unhandled_exception()
    {
        print(PRI2, "async_generator_promise_base::rethrow_if_unhandled_exception()\n");
        if (m_exception)
        {
            std::rethrow_exception(std::move(m_exception));
        }
    }

protected:
    async_generator_yield_operation internal_yield_value() noexcept;

private:
    friend class async_generator_yield_operation;
    friend class async_generator_advance_operation;

    std::exception_ptr m_exception;

    std::coroutine_handle<> m_consumerCoroutine;

protected:
    void* m_currentValue;
};

//--------------------------------------------------------------

class async_generator_yield_operation final
{
public:
    async_generator_yield_operation(std::coroutine_handle<> consumer) noexcept
        : m_consumer(consumer)
    {
        print(PRI2, "async_generator_yield_operation::async_generator_yield_operation(std::coroutine_handle<> consumer)\n");
    }

    bool await_ready() const noexcept
    {
        print(PRI2, "async_generator_yield_operation::await_ready()\n");
        return false;
    }

    std::coroutine_handle<>
        await_suspend(std::coroutine_handle<> producer) noexcept
    {
        print(PRI2, "async_generator_yield_operation::await_suspend(std::coroutine_handle<> producer)\n");
        return m_consumer;
    }

    void await_resume() noexcept
    {
        print(PRI2, "async_generator_yield_operation::await_resume()\n");
    }

private:
    std::coroutine_handle<> m_consumer;
};

//--------------------------------------------------------------

inline async_generator_yield_operation async_generator_promise_base::final_suspend() noexcept
{
    print(PRI2, "async_generator_promise_base::final_suspend()\n");
    m_currentValue = nullptr;
    return internal_yield_value();
}

inline async_generator_yield_operation async_generator_promise_base::internal_yield_value() noexcept
{
    print(PRI2, "async_generator_promise_base::internal_yield_value()\n");
    return async_generator_yield_operation{ m_consumerCoroutine };
}

//--------------------------------------------------------------

class async_generator_advance_operation
{
protected:
    async_generator_advance_operation(std::nullptr_t) noexcept
        : m_promise(nullptr)
        , m_producerCoroutine(nullptr)
    {
        print(PRI2, "async_generator_advance_operation::async_generator_advance_operation(std::nullptr_t)\n");
    }

    async_generator_advance_operation(
        async_generator_promise_base& promise,
        std::coroutine_handle<> producerCoroutine) noexcept
        : m_promise(std::addressof(promise))
        , m_producerCoroutine(producerCoroutine)
    {
        print(PRI2, "async_generator_advance_operation::async_generator_advance_operation(...)\n");
    }

public:
    bool await_ready() const noexcept
    {
        print(PRI2, "async_generator_advance_operation::await_ready()\n");
        return false;
    }

    std::coroutine_handle<>
        await_suspend(std::coroutine_handle<> consumerCoroutine) noexcept
    {
        print(PRI2, "async_generator_advance_operation::await_suspend(std::coroutine_handle<> consumerCoroutine)\n");
        m_promise->m_consumerCoroutine = consumerCoroutine;
        return m_producerCoroutine;
    }

protected:
    async_generator_promise_base* m_promise;
    std::coroutine_handle<> m_producerCoroutine;
};

//--------------------------------------------------------------

template<typename T>
class async_generator_promise final : public async_generator_promise_base
{
    using value_type = std::remove_reference_t<T>;
public:
    async_generator_promise() noexcept = default;

    async_generator<T> get_return_object() noexcept;

    async_generator_yield_operation yield_value(value_type& value) noexcept
    {
        print(PRI2, "async_generator_promise::yield_value(value_type& value)\n");
        m_currentValue = std::addressof(value);
        return internal_yield_value();
    }

    async_generator_yield_operation yield_value(value_type&& value) noexcept
    {
        print(PRI2, "async_generator_promise::yield_value(value_type&& value)\n");
        return yield_value(value);
    }

    T& value() const noexcept
    {
        print(PRI2, "async_generator_promise::value()\n");
        return *static_cast<T*>(m_currentValue);
    }
};

//--------------------------------------------------------------

template<typename T>
class async_generator_promise<T&&> final : public async_generator_promise_base
{
public:

    async_generator_promise() noexcept = default;

    async_generator<T> get_return_object() noexcept;

    async_generator_yield_operation yield_value(T&& value) noexcept
    {
        print(PRI2, "async_generator_promise<T&&>::yield_value(T&& value)\n");
        m_currentValue = std::addressof(value);
        return internal_yield_value();
    }

    T&& value() const noexcept
    {
        print(PRI2, "async_generator_promise<T&&>::value()\n");
        return std::move(*static_cast<T*>(m_currentValue));
    }
};

//--------------------------------------------------------------

template<typename T>
class async_generator_increment_operation final : public async_generator_advance_operation
{
public:
    async_generator_increment_operation(async_generator_iterator<T>& iterator) noexcept
        : async_generator_advance_operation(iterator.m_coroutine.promise(), iterator.m_coroutine)
        , m_iterator(iterator)
    {
        print(PRI2, "async_generator_increment_operation::async_generator_increment_operation(async_generator_iterator<T>& iterator)\n");
    }

    async_generator_iterator<T>& await_resume();

private:
    async_generator_iterator<T>& m_iterator;
};

//--------------------------------------------------------------

template<typename T>
class async_generator_iterator final
{
    using promise_type = async_generator_promise<T>;
    using handle_type = std::coroutine_handle<promise_type>;

public:
    using iterator_category = std::input_iterator_tag;
    // Not sure what type should be used for difference_type as we don't
    // allow calculating difference between two iterators.
    using difference_type = std::ptrdiff_t;
    using value_type = std::remove_reference_t<T>;
    using reference = std::add_lvalue_reference_t<T>;
    using pointer = std::add_pointer_t<value_type>;

    async_generator_iterator(std::nullptr_t) noexcept
        : m_coroutine(nullptr)
    {
        print(PRI2, "async_generator_iterator::async_generator_iterator(std::nullptr_t)\n");
    }

    async_generator_iterator(handle_type coroutine) noexcept
        : m_coroutine(coroutine)
    {
        print(PRI2, "async_generator_iterator::async_generator_iterator(handle_type coroutine)\n");
    }

    async_generator_increment_operation<T> operator++() noexcept
    {
        print(PRI2, "async_generator_iterator::operator++()\n");
        return async_generator_increment_operation<T>{ *this };
    }

    reference operator*() const noexcept
    {
        print(PRI2, "async_generator_iterator::operator*()\n");
        return m_coroutine.promise().value();
    }

    bool operator==(const async_generator_iterator& other) const noexcept
    {
        print(PRI2, "async_generator_iterator::operator==(const async_generator_iterator& other)\n");
        return m_coroutine == other.m_coroutine;
    }

    bool operator!=(const async_generator_iterator& other) const noexcept
    {
        print(PRI2, "async_generator_iterator::operator!=(const async_generator_iterator& other)\n");
        return !(*this == other);
    }

private:
    friend class async_generator_increment_operation<T>;

    handle_type m_coroutine;
};

//--------------------------------------------------------------

template<typename T>
async_generator_iterator<T>& async_generator_increment_operation<T>::await_resume()
{
    print(PRI2, "async_generator_increment_operation<T>::await_resume()\n");
    if (m_promise->finished())
    {
        // Update iterator to end()
        m_iterator = async_generator_iterator<T>{ nullptr };
        m_promise->rethrow_if_unhandled_exception();
    }

    return m_iterator;
}

//--------------------------------------------------------------

template<typename T>
class async_generator_begin_operation final : public async_generator_advance_operation
{
    using promise_type = async_generator_promise<T>;
    using handle_type = std::coroutine_handle<promise_type>;

public:
    async_generator_begin_operation(std::nullptr_t) noexcept
        : async_generator_advance_operation(nullptr)
    {
        print(PRI2, "async_generator_begin_operation::async_generator_begin_operation(std::nullptr_t)\n");
    }

    async_generator_begin_operation(handle_type producerCoroutine) noexcept
        : async_generator_advance_operation(producerCoroutine.promise(), producerCoroutine)
    {
        print(PRI2, "async_generator_begin_operation::async_generator_begin_operation(handle_type producerCoroutine)\n");
    }

    bool await_ready() const noexcept
    {
        print(PRI2, "async_generator_begin_operation::await_ready()\n");
        return m_promise == nullptr || async_generator_advance_operation::await_ready();
    }

    async_generator_iterator<T> await_resume()
    {
        print(PRI2, "async_generator_begin_operation::await_resume()\n");
        if (m_promise == nullptr)
        {
            // Called begin() on the empty generator.
            return async_generator_iterator<T>{ nullptr };
        }
        else if (m_promise->finished())
        {
            // Completed without yielding any values.
            m_promise->rethrow_if_unhandled_exception();
            return async_generator_iterator<T>{ nullptr };
        }

        return async_generator_iterator<T>{
            handle_type::from_promise(*static_cast<promise_type*>(m_promise))
        };
    }
};

//--------------------------------------------------------------

template<typename T>
class async_generator
{
public:
    using promise_type = async_generator_promise<T>;
    using iterator = async_generator_iterator<T>;

    async_generator() noexcept
        : m_coroutine(nullptr)
    {
        print(PRI2, "async_generator::async_generator()\n");
    }

    explicit async_generator(promise_type& promise) noexcept
        : m_coroutine(std::coroutine_handle<promise_type>::from_promise(promise))
    {
        print(PRI2, "async_generator::async_generator(promise_type& promise)\n");
    }

    async_generator(async_generator&& other) noexcept
        : m_coroutine(other.m_coroutine)
    {
        print(PRI2, "async_generator::async_generator(async_generator&& other)\n");
        other.m_coroutine = nullptr;
    }

    ~async_generator()
    {
        print(PRI2, "async_generator::~async_generator()\n");
        if (m_coroutine)
        {
            m_coroutine.destroy();
        }
    }

    async_generator& operator=(async_generator&& other) noexcept
    {
        print(PRI2, "async_generator::operator=(async_generator&& other)\n");
        async_generator temp(std::move(other));
        swap(temp);
        return *this;
    }

    async_generator(const async_generator&) = delete;
    async_generator& operator=(const async_generator&) = delete;

    auto begin() noexcept
    {
        print(PRI2, "async_generator::begin()\n");
        if (!m_coroutine)
        {
            return async_generator_begin_operation<T>{ nullptr };
        }

        return async_generator_begin_operation<T>{ m_coroutine };
    }

    auto end() noexcept
    {
        print(PRI2, "async_generator::end()\n");
        return iterator{ nullptr };
    }

    void swap(async_generator& other) noexcept
    {
        print(PRI2, "async_generator::swap(async_generator& other)\n");
        using std::swap;
        swap(m_coroutine, other.m_coroutine);
    }

private:
    std::coroutine_handle<promise_type> m_coroutine;
};

//--------------------------------------------------------------

template<typename T>
void swap(async_generator<T>& a, async_generator<T>& b) noexcept
{
    print(PRI2, "swap(async_generator<T>& a, async_generator<T>& b)\n");
    a.swap(b);
}


template<typename T>
async_generator<T> async_generator_promise<T>::get_return_object() noexcept
{
    print(PRI2, "async_generator_promise<T>::get_return_object()\n");
    return async_generator<T>{ *this };
}

//--------------------------------------------------------------

template<typename T>
struct sync {
    struct promise_type;
    friend struct promise_type;
    using handle_type = std::coroutine_handle<promise_type>;

    sync(const sync& s) = delete;

    sync(sync&& s)
        : coro(s.coro) {
        print(PRI2, "sync::sync(sync&& s)\n");
        s.coro = nullptr;
    }

    ~sync() {
        print(PRI2, "sync::~sync()\n");
        //if (coro) coro.destroy();    // can throw exception when sync goes out of scope. FFS
    }

    sync& operator = (const sync&) = delete;

    sync& operator = (sync&& s) {
        //print(PRI2, "sync::sync = (sync&& s)\n");
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }

    T get() {
        print(PRI2, "sync::get()\n");
        while (!coro.promise().set)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return coro.promise().value;
    }

    bool await_ready() {
        print(PRI2, "sync::await_ready(): return false;\n");
        //const auto ready = this->coro.done();
        return false;
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        print(PRI2, "sync::await_suspend(std::coroutine_handle<> awaiting)\n");
        this->coro.promise().m_awaiting = awaiting;
    }

    auto await_resume() {
        print(PRI2, "sync::await_resume()\n");
        const auto r = this->coro.promise().value;
        print(PRI2, "sync::await_resume(): this->coro.promise().set = %d\n", this->coro.promise().set);
        return r;
    }

    struct promise_type {

        friend struct sync;

        promise_type() : set(false) {
            print(PRI2, "sync::promise_type::promise_type()\n");
        }

        ~promise_type() {
            print(PRI2, "sync::promise_type::~promise_type()\n");
        }

        auto return_value(T v) {
            print(PRI2, "sync::promise_type::return_value(T v)\n");
            this->value = v;
            this->set = true;
            if (this->m_awaiting)
                this->m_awaiting.resume();
            else
                print(PRI2, "sync::promise_type::return_value(T v): this->m_awaiting == nullptr\n");
        }

        auto get_return_object() {
            print(PRI2, "sync::promise_type::get_return_object()\n");
            return sync<T>{handle_type::from_promise(*this)};
        }

        auto initial_suspend() {
            print(PRI2, "sync::promise_type::initial_suspend()\n");
            return std::suspend_never{};
        }

        auto final_suspend() noexcept {
            print(PRI2, "sync::promise_type::final_suspend()\n");
            return std::suspend_always{};
        }

        void unhandled_exception() {
            print(PRI2, "sync::promise::promise_type()\n");
            std::exit(1);
        }

    private:
        T value;
        bool set;
        std::coroutine_handle<> m_awaiting;
    };

    sync(handle_type h)
        : coro(h) {
        print(PRI2, "sync::sync(handle_type h)\n");
    }

    handle_type coro;
};

//--------------------------------------------------------------
//--------------------------------------------------------------

async_generator<std::uint32_t> makeSequence()
{
    print(PRI1, "makeSequence()\n");

    for (std::uint32_t i = 0; i < 1500u; ++i)
    //for (std::uint32_t i = 0; i < 10u; ++i)
    {
        print(); print(PRI1, "makeSequence(): co_yield i = %d;\n", i);
        co_yield i;
    }
}

sync<int> test()
{
    print(PRI1, "test()\n");

    auto sequence = makeSequence();

    std::uint32_t expected = 0;
    for co_await(std::uint32_t i : sequence)
    {
        print(); print(PRI1, "test(): i = %d, expected++ = %d\n", i, expected++);
    }

    co_return 0;
}

int main()
{
    print(PRI1, "main(): sync<int> t = test();\n");
    sync<int> t = test();
    print(PRI1, "main(): t.get();\n");
    t.get();

    print(PRI1, "main(): return 0;\n");
    return 0;
}
