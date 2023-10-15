// Source: https://godbolt.org/z/xaj3Yxabn

////////////////////////////////////////////////////////////////////////////////////////
// Supporting code for blog post "C++ Coroutines: Understanding the Compiler Transform"
//
// https://lewissbaker.github.io/2022/08/27/understanding-the-compiler-transform
// 
// By Lewis Baker
////////////////////////////////////////////////////////////////////////////////////////

#ifndef _LBCOROUTINE_H_
#define _LBCOROUTINE_H_

#include <type_traits>
#include <memory>           // std::addressof

//////////////////////////////////////////////////
// <coroutine> header definitions
//
// This section contains definitions of the parts of <coroutine> header needed by
// the lowering and implementation below.

struct __coroutine_state {
    using __resume_fn = __coroutine_state * (__coroutine_state*);
    using __destroy_fn = void(__coroutine_state*);

    __resume_fn* __resume;
    __destroy_fn* __destroy;

    static __coroutine_state* __noop_resume(__coroutine_state* __state) noexcept {
        return __state;
    }

    static void __noop_destroy(__coroutine_state*) noexcept {}

    static const __coroutine_state __noop_coroutine;
};

inline const __coroutine_state __coroutine_state::__noop_coroutine{
    &__coroutine_state::__noop_resume,
    &__coroutine_state::__noop_destroy
};

template<typename Promise>
struct __coroutine_state_with_promise : __coroutine_state {
    __coroutine_state_with_promise() noexcept {}
    ~__coroutine_state_with_promise() {}

    union {
        Promise __promise;
    };
};

namespace std
{
    template<typename Ret, typename... Args>
    struct coroutine_traits
    {
        using promise_type = typename std::remove_cvref_t<Ret>::promise_type;
    };

    template<typename Promise = void>
    class coroutine_handle;

    template<>
    class coroutine_handle<void> {
    public:
        coroutine_handle() noexcept = default;
        coroutine_handle(const coroutine_handle&) noexcept = default;
        coroutine_handle& operator=(const coroutine_handle&) noexcept = default;

        coroutine_handle(nullptr_t) noexcept {}     // Added by JVS to allow nullptr

        void* address() const {
            return static_cast<void*>(state_);
        }

        static coroutine_handle from_address(void* ptr) {
            coroutine_handle h;
            h.state_ = static_cast<__coroutine_state*>(ptr);
            return h;
        }

        explicit operator bool() noexcept {
            return state_ != nullptr;
        }

        coroutine_handle& operator=(nullptr_t) noexcept {       // Added by JVS to allow nullptr
            state_ = nullptr;
            return *this;
        }

        friend bool operator==(coroutine_handle a, coroutine_handle b) noexcept {
            return a.state_ == b.state_;
        }
#if 0
        void resume() const {
            __coroutine_state* s = state_;
            do {
                s = s->__resume(s);
            } while (s != &__coroutine_state::__noop_coroutine);
        }
#else
        void resume() const {
            __coroutine_state* s = state_;
            if (s) {
                do {
                    s = s->__resume(s);
                } while (s != nullptr && s != &__coroutine_state::__noop_coroutine);
            }
        }
#endif
        void destroy() const {
            state_->__destroy(state_);
        }

        bool done() const {
            return state_->__resume == nullptr;
        }

    private:
        __coroutine_state* state_ = nullptr;
    };

    template<typename Promise>
    class coroutine_handle {
        using state_t = __coroutine_state_with_promise<Promise>;
    public:
        coroutine_handle() noexcept = default;
        coroutine_handle(const coroutine_handle&) noexcept = default;
        coroutine_handle& operator=(const coroutine_handle&) noexcept = default;

        operator coroutine_handle<void>() const noexcept {
            return coroutine_handle<void>::from_address(address());
        }

        explicit operator bool() const noexcept {
            return state_ != nullptr;
        }

        friend bool operator==(coroutine_handle a, coroutine_handle b) noexcept {
            return a.state_ == b.state_;
        }

        void* address() const {
            return static_cast<void*>(static_cast<__coroutine_state*>(state_));
        }

        static coroutine_handle from_address(void* ptr) {
            coroutine_handle h;
            h.state_ = static_cast<state_t*>(static_cast<__coroutine_state*>(ptr));
            return h;
        }

        Promise& promise() const {
            return state_->__promise;
        }

        static coroutine_handle from_promise(Promise& promise) {
            coroutine_handle h;

            // We know the address of the __promise member, so calculate the
            // address of the coroutine-state by subtracting the offset of
            // the __promise field from this address.
            h.state_ = reinterpret_cast<state_t*>(
                reinterpret_cast<unsigned char*>(std::addressof(promise)) -
                offsetof(state_t, __promise));

            return h;
        }

        // Define these in terms of their `coroutine_handle<void>` implementations

        void resume() const {
            static_cast<coroutine_handle<void>>(*this).resume();
        }

        void destroy() const {
            static_cast<coroutine_handle<void>>(*this).destroy();
        }

        bool done() const {
            return static_cast<coroutine_handle<void>>(*this).done();
        }

    private:
        state_t* state_;
    };

    struct noop_coroutine_promise {};

    using noop_coroutine_handle = coroutine_handle<noop_coroutine_promise>;

    noop_coroutine_handle noop_coroutine() noexcept;

    template<>
    class coroutine_handle<noop_coroutine_promise> {
    public:
        constexpr coroutine_handle(const coroutine_handle&) noexcept = default;
        constexpr coroutine_handle& operator=(const coroutine_handle&) noexcept = default;

        constexpr explicit operator bool() noexcept { return true; }

        constexpr friend bool operator==(coroutine_handle, coroutine_handle) noexcept {
            return true;
        }

        operator coroutine_handle<void>() const noexcept {
            return coroutine_handle<void>::from_address(address());
        }

        noop_coroutine_promise& promise() const noexcept {
            static noop_coroutine_promise promise;
            return promise;
        }

        constexpr void resume() const noexcept {}
        constexpr void destroy() const noexcept {}
        constexpr bool done() const noexcept { return false; }

        constexpr void* address() const noexcept {
            return const_cast<__coroutine_state*>(&__coroutine_state::__noop_coroutine);
        }
    private:
        constexpr coroutine_handle() noexcept = default;

        friend noop_coroutine_handle noop_coroutine() noexcept {
            return {};;
        }
    };

    struct suspend_always {
        constexpr suspend_always() noexcept = default;
        constexpr bool await_ready() const noexcept { return false; }
        constexpr void await_suspend(coroutine_handle<>) const noexcept {}
        constexpr void await_resume() const noexcept {}
    };

    // Added by JVS
    struct suspend_never {
        constexpr suspend_never() noexcept = default;
        constexpr bool await_ready() const noexcept { return true; }
        constexpr void await_suspend(coroutine_handle<>) const noexcept {}
        constexpr void await_resume() const noexcept {}
    };
}

#endif
