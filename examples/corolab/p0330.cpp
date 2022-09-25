/**
 *  Filename: p0330.cpp
 *  Description: Illustrates use of co_yield to define an asynchronous generator type.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *  Based upon: https://github.com/kirkshoop/await
 */

#include <iostream>
#include <future>
#include <string>
#include <chrono>

#include <functional>
#include <cassert>
#include <atomic>
#include <iostream>
#include <thread>

#include <stdarg.h>

const int priority = 0x0F;

#include "print.h"

//--------------------------------------------------------------

using namespace std::chrono;
using clk = system_clock;
using namespace std::chrono_literals;

#include <coroutine>
#include <generator>

using namespace std;

#include <windows.h>
#include <threadpoolapiset.h>

namespace rx {

template <typename _Ty, typename _GeneratorPromise, typename _Alloc>
struct await_iterator;

template <typename _Ty, typename _GeneratorPromise, typename _Alloc>
struct async_iterator;

template <typename _Ty, typename _GeneratorPromise, typename _Alloc >
struct await_consumer;

template <typename _Ty, typename _Alloc = allocator<char> >
struct async_generator
{
    struct promise_type
    {
        coroutine_handle<> _AwaitIteratorCoro;
        coroutine_handle<> _AwaitConsumerCoro;
        const _Ty* _CurrentValue;

        promise_type& get_return_object()
        {
            print(PRI2, "async_generator::promise_type::get_return_object()\n");
            return *this;
        }

        suspend_always initial_suspend()
        {
            print(PRI2, "async_generator::promise_type::initial_suspend()\n");
            return{};
        }

        suspend_always final_suspend() noexcept
        {
            print(PRI2, "async_generator::promise_type::final_suspend()\n");
            return{};
        }

        await_consumer<_Ty, promise_type, _Alloc> yield_value(_Ty const & _Value)
        {
            print(PRI2, "async_generator::promise_type::yield_value(_Ty const & _Value)\n");
            _CurrentValue = _STD addressof(_Value);
            return{ coroutine_handle<promise_type>::from_promise(*this) };
        }

        void return_void()
        {
            print(PRI2, "async_generator::promise_type::return_void()\n");
            auto _Coro = move(_AwaitIteratorCoro);
            _AwaitIteratorCoro = nullptr;
            if (_Coro) {
                _Coro.resume();
            }
        }

        using _Alloc_traits = allocator_traits<_Alloc>;
        using _Alloc_of_char_type = typename _Alloc_traits::template rebind_alloc<char>;

        void* operator new(size_t _Size)
        {
            print(PRI2, "async_generator::promise_type::operator new(size_t _Size)\n");
            _Alloc_of_char_type _Al;
            return _Al.allocate(_Size);
        }

        void operator delete(void* _Ptr, size_t _Size) noexcept
        {
            print(PRI2, "async_generator::promise_type::operator delete(void* _Ptr, size_t _Size)\n");
            _Alloc_of_char_type _Al;
            return _Al.deallocate(static_cast<char*>(_Ptr), _Size);
        }

        void unhandled_exception()
        {
        }
    };

    await_iterator<_Ty, promise_type, _Alloc> begin()
    {
        print(PRI2, "async_generator::begin()\n");
        return _Coro;
    }

    async_iterator<_Ty, promise_type, _Alloc> end()
    {
        print(PRI2, "async_generator::end()\n");
        return{ nullptr };
    }

    explicit async_generator(promise_type& _Prom)
        : _Coro(coroutine_handle<promise_type>::from_promise(_Prom))
    {
        print(PRI2, "async_generator::async_generator(promise_type& _Prom)\n");
    }

    async_generator() = default;

    async_generator(async_generator const&) = delete;
    async_generator& operator = (async_generator const&) = delete;

    async_generator(async_generator && _Right)
        : _Coro(_Right._Coro)
    {
        print(PRI2, "async_generator::async_generator(async_generator && _Right)\n");
        _Right._Coro = nullptr;
    }

    async_generator& operator = (async_generator && _Right)
    {
        print(PRI2, "async_generator::operator = (async_generator && _Right)\n");
        if (&_Right != this)
        {
            _Coro = _Right._Coro;
            _Right._Coro = nullptr;
        }
    }

    ~async_generator()
    {
        print(PRI2, "async_generator::~async_generator()\n");
        if (_Coro)
        {
            _Coro.destroy();
        }
    }
private:
    coroutine_handle<promise_type> _Coro = nullptr;
};

template <typename _Ty, typename _GeneratorPromise, typename _Alloc >
struct await_consumer
{
    coroutine_handle<_GeneratorPromise> _GeneratorCoro;

    await_consumer(coroutine_handle<_GeneratorPromise> _GCoro)
        : _GeneratorCoro(_GCoro)
    {
        print(PRI2, "await_consumer::await_consumer(coroutine_handle<_GeneratorPromise> _GCoro)\n");
    }

    await_consumer() = default;

    await_consumer(await_consumer const&) = delete;
    await_consumer& operator = (await_consumer const&) = delete;

    await_consumer(await_consumer && _Right)
        : _GeneratorCoro(_Right._GeneratorCoro)
    {
        print(PRI2, "await_consumer::await_consumer(await_consumer && _Right)\n");
        _Right._GeneratorCoro = nullptr;
    }

    await_consumer& operator = (await_consumer && _Right)
    {
        print(PRI2, "await_consumer::operator = (await_consumer && _Right)\n");
        if (&_Right != this)
        {
            _GeneratorCoro = _Right._GeneratorCoro;
            _Right._GeneratorCoro = nullptr;
        }
        return *this;
    }

    ~await_consumer()
    {
        print(PRI2, "await_consumer::~await_consumer()\n");
    }

    bool await_ready() noexcept
    {
        print(PRI2, "await_consumer::await_ready()\n");
        return false;
    }

    void await_suspend(coroutine_handle<> _AwaitConsumerCoro) noexcept
    {
        print(PRI2, "await_consumer::await_suspend(coroutine_handle<> _AwaitConsumerCoro)\n");

        _GeneratorCoro.promise()._AwaitConsumerCoro = _AwaitConsumerCoro;

        auto _AwaitIteratorCoro = move(_GeneratorCoro.promise()._AwaitIteratorCoro);
        _GeneratorCoro.promise()._AwaitIteratorCoro = nullptr;
        if (_AwaitIteratorCoro) {
            _AwaitIteratorCoro.resume();
        }
    }

    void await_resume() noexcept
    {
        print(PRI2, "await_consumer::await_resume()\n");
    }
};

template <typename _Ty, typename _GeneratorPromise, typename _Alloc >
struct await_iterator 
{
    coroutine_handle<_GeneratorPromise> _GeneratorCoro;
    async_iterator<_Ty, _GeneratorPromise, _Alloc>* _It;
    bool owner;

    await_iterator(coroutine_handle<_GeneratorPromise> _GCoro)
        : _GeneratorCoro(_GCoro)
        , _It(nullptr)
    {
        print(PRI2, "await_iterator::await_iterator(coroutine_handle<_GeneratorPromise> _GCoro)\n");
    }

    // operator++ needs to update itself
    await_iterator(async_iterator<_Ty, _GeneratorPromise, _Alloc>* _OIt)
        : _GeneratorCoro(_OIt->_GeneratorCoro)
        , _It(_OIt)
    {
        print(PRI2, "await_iterator::await_iterator(async_iterator<_Ty, _GeneratorPromise, _Alloc>* _OIt)\n");
    }

    await_iterator() 
        : _GeneratorCoro()
        , _It(nullptr)
    {
        print(PRI2, "await_iterator::await_iterator()\n");
    }

    await_iterator(await_iterator const&) = delete;
    await_iterator& operator = (await_iterator const&) = delete;

    await_iterator(await_iterator && _Right)
        : _GeneratorCoro(_Right._GeneratorCoro)
        , _It(_Right)
    {
        print(PRI2, "await_iterator::await_iterator(await_iterator && _Right)\n");
        _Right._GeneratorCoro = nullptr;
        _Right._It = nullptr;
    }

    await_iterator& operator = (await_iterator && _Right)
    {
        print(PRI2, "await_iterator::operator = (await_iterator && _Right)\n");
        if (&_Right != this)
        {
            _GeneratorCoro = _Right._GeneratorCoro;
            _Right._GeneratorCoro = nullptr;

            _It = _Right._It;
            _Right._It = nullptr;
        }
        return *this;
    }

    ~await_iterator()
    {
        print(PRI2, "await_iterator::~await_iterator()\n");
    }

    bool await_ready() noexcept
    {
        print(PRI2, "await_iterator::await_ready()\n");
        return false;
    }

    void await_suspend(coroutine_handle<> _AwaitIteratorCoro) noexcept
    {
        print(PRI2, "await_iterator::await_suspend(coroutine_handle<> _AwaitIteratorCoro)\n");
        _GeneratorCoro.promise()._AwaitIteratorCoro = _AwaitIteratorCoro;

        auto _AwaitConsumerCoro = move(_GeneratorCoro.promise()._AwaitConsumerCoro);
        _GeneratorCoro.promise()._AwaitConsumerCoro = nullptr;
        if (_AwaitConsumerCoro) {
            // resume co_yield
            _GeneratorCoro.promise()._CurrentValue = nullptr;
            _AwaitConsumerCoro.resume();
        }
        else {
            // first resume
            _GeneratorCoro.resume();
        }

    }

    async_iterator<_Ty, _GeneratorPromise, _Alloc> await_resume() noexcept
    {
        print(PRI2, "await_iterator::await_resume()\n");
        if (_GeneratorCoro.done() || !_GeneratorCoro.promise()._CurrentValue) {
            _GeneratorCoro = nullptr;
        }
        if (_It) {
            _It->_GeneratorCoro = _GeneratorCoro;
            return{*_It};
        }
        return{ _GeneratorCoro };
    }
};

template <typename _Ty, typename _GeneratorPromise, typename _Alloc>
struct async_iterator
    : _STD iterator<input_iterator_tag, _Ty>
{
    coroutine_handle<_GeneratorPromise> _GeneratorCoro;

    async_iterator(nullptr_t)
        : _GeneratorCoro(nullptr)
    {
        print(PRI2, "async_iterator::async_iterator(nullptr_t)\n");
    }

    async_iterator(coroutine_handle<_GeneratorPromise> _GCoro)
        : _GeneratorCoro(_GCoro)
    {
        print(PRI2, "async_iterator::async_iterator(coroutine_handle<_GeneratorPromise> _GCoro)\n");
    }

    await_iterator<_Ty, _GeneratorPromise, _Alloc> operator++()
    {
        print(PRI2, "async_iterator::operator++()\n");
        if (!_GeneratorCoro) abort();
        return{this};
    }

    async_iterator operator++(int) = delete;
    // generator iterator current_value
    // is a reference to a temporary on the coroutine frame
    // implementing postincrement will require storing a copy
    // of the value in the iterator.
    //{
    //	auto _Result = *this;
    //	++(*this);
    //	return _Result;
    //}

    bool operator==(async_iterator const& _Right) const
    {
        print(PRI2, "async_iterator::operator==(async_iterator const& _Right)\n");
        return _GeneratorCoro == _Right._GeneratorCoro;
    }

    bool operator!=(async_iterator const& _Right) const
    {
        print(PRI2, "async_iterator::operator!=(async_iterator const& _Right)\n");
        return !(*this == _Right);
    }

    _Ty const& operator*() const
    {
        print(PRI2, "async_iterator::operator*()\n");
        return *_GeneratorCoro.promise()._CurrentValue;
    }

    _Ty const* operator->() const
    {
        print(PRI2, "async_iterator::operator->()\n");
        return _STD addressof(operator*());
    }

};

}

// usage: await resume_at(std::chrono::system_clock::now() + 1s);
auto resume_at(clk::time_point at)
{
    print(PRI2, "resume_at(clk::time_point at)\n");

    class awaiter
    {
        static void CALLBACK TimerCallback(PTP_CALLBACK_INSTANCE, void* Context, PTP_TIMER)
        {
            print(PRI2, "awaiter::TimerCallback(PTP_CALLBACK_INSTANCE, void* Context, PTP_TIMER)\n");
            coroutine_handle<>::from_address(Context)();
        }
        PTP_TIMER timer = nullptr;
        clk::time_point at;

    public:
        awaiter(clk::time_point a)
            : at(a)
        {
            print(PRI2, "awaiter::awaiter(clk::time_point a)\n");
        }

        bool await_ready() const
        {
            print(PRI2, "awaiter::await_ready()\n");
            return clk::now() >= at;
        }

        void await_suspend(coroutine_handle<> resume_cb)
        {
            print(PRI2, "awaiter::await_suspend(coroutine_handle<> resume_cb)\n");
            auto duration = at - clk::now();
            int64_t relative_count = -duration.count();
            timer = CreateThreadpoolTimer(TimerCallback, resume_cb.address(), nullptr);
            if (timer == 0)
                throw system_error(GetLastError(), system_category());
            SetThreadpoolTimer(timer, (PFILETIME)&relative_count, 0, 0);
        }

        void await_resume()
        {
            print(PRI2, "awaiter::await_resume()\n");
        }

        ~awaiter()
        {
            print(PRI2, "awaiter::~awaiter()\n");
            if (timer) CloseThreadpoolTimer(timer);
        }
    };

    return awaiter{ at };
}

// usage: await resume_after(1s);
auto resume_after(clk::duration period)
{
    print(PRI2, "resume_after(clk::duration period)\n");
    return resume_at(clk::now() + period);
}

rx::async_generator<int> fibonacci(int n)
{
    int a = 0;
    int b = 1;

    print(PRI2, "fibonacci(int %d): while (n-- > 0)\n", n);
    while (n-- > 0) {
        print(PRI2, "fibonacci(int %d): co_yield %d;\n", n, a);
        co_yield a;
        auto next = a + b;
        a = b;
        b = next;
    }
    print(PRI2, "fibonacci(int %d): return\n");
}

template<class Adaptor>
struct adaptor
{
    mutable decay_t<Adaptor> a;

    template<class T, class Alloc>
    auto operator()(rx::async_generator<T, Alloc> s) const -> 
        result_of_t<decay_t<Adaptor>(rx::async_generator<T, Alloc>)>
    {
        print(PRI2, "adaptor::operator()(rx::async_generator<T, Alloc> s)\n");
        return a(move(s));
    }
};

template<class Adaptor>
auto make_adaptor(Adaptor&& a) -> adaptor<decay_t<Adaptor>>
{
    print(PRI2, "make_adaptor(Adaptor&& a)\n");
    return{ forward<Adaptor>(a) };
}

namespace detail {

    struct delay
    {
        clk::duration period;

        template<class T, class Alloc>
        auto operator()(rx::async_generator<T, Alloc> s) const -> rx::async_generator<T, Alloc>
        {
            print(PRI2, "delay::operator()(rx::async_generator<T, Alloc> s): for co_await(auto i : s)\n");
            for co_await(auto i : s)
            {
                print(PRI2, "delay::operator()(rx::async_generator<T, Alloc> s): co_await resume_after(period);\n");
                co_await resume_after(period);
                co_yield i;
            }
            print(PRI2, "delay::operator()(rx::async_generator<T, Alloc> s): return;\n");
        }
    };
}

auto delay(clk::duration p) -> adaptor<detail::delay>
{
    print(PRI2, "delay(clk::duration p)\n");
    return make_adaptor(detail::delay{ p });
}

namespace detail
{
    template<class Pred>
    struct copy_if
    {
        mutable decay_t<Pred> pred;

        template<class T, class Alloc>
        auto operator()(rx::async_generator<T, Alloc> s) const -> rx::async_generator<T, Alloc>
        {
            print(PRI2, "copy_if::operator()(rx::async_generator<T, Alloc> s): for co_await(auto i : s)\n");
            for co_await(auto i : s)
            {
                if (pred(i)) {
                    print(PRI2, "copy_if::operator()(rx::async_generator<T, Alloc> s): co_yield %d;\n", i);
                    co_yield i;
                }
            }
            print(PRI2, "copy_if::operator()(rx::async_generator<T, Alloc> s): return;\n");
        }
    };
}

template<class Pred>
auto copy_if(Pred&& p) -> adaptor<detail::copy_if<Pred>>
{
    print(PRI2, "copy_if(Pred&& p)\n");
    return make_adaptor(detail::copy_if<Pred>{forward<Pred>(p)});
}

namespace detail
{
    template<class Transform>
    struct transform
    {
        mutable decay_t<Transform> t;

        template<class T, class Alloc>
        auto operator()(rx::async_generator<T, Alloc> s) const -> 
            rx::async_generator<result_of_t<decay_t<Transform>(T)>, Alloc>
        {
            print(PRI2, "transform::operator()(rx::async_generator<T, Alloc> s): for co_await(auto v : s)\n");
            for co_await(auto v : s)
            {
                print(PRI2, "transform::operator()(rx::async_generator<T, Alloc> s): co_yield t(v);\n");
                co_yield t(v);
            }
            print(PRI2, "transform::operator()(rx::async_generator<T, Alloc> s): return;\n");
        }
    };
}

template<class Pred>
auto transform(Pred&& p) -> adaptor<detail::transform<Pred>>
{
    print(PRI2, "transform(Pred&& p)\n");
    return make_adaptor(detail::transform<Pred>{forward<Pred>(p)});
}

template<class T, class Alloc, class Adaptor>
auto operator|(rx::async_generator<T, Alloc> s, adaptor<Adaptor> adapt) -> 
    result_of_t<decay_t<Adaptor>(rx::async_generator<T, Alloc>)>
{
    print(PRI2, "operator|(rx::async_generator<T, Alloc> s, adaptor<Adaptor> adapt)\n");
    return adapt(move(s));
}

future<void> waitfor()
{
    print(PRI2, "waitfor(): co_await(auto v : fibonacci(10)\n");
    for co_await(auto v : fibonacci(10) 
        | copy_if([](int v) { return v % 2 != 0; })
        | transform([](int i) { return i; })
//      | transform([](int i) { return to_string(i) + ","; })
//      | delay(1s)     // makes application hang
    )
    {
        print(PRI2, "waitfor(): v = %d\n", v);
    }
    print(PRI2, "waitfor(): return\n");
}

int main() {
    print(PRI2, "main(): waitfor().get();\n");
    waitfor().get();
    print(PRI2, "main(): return 0;\n");
    return 0;
}
