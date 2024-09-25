// Source: https://godbolt.org/z/xaj3Yxabn

////////////////////////////////////////////////////////////////////////////////////////
// Supporting code for blog post "C++ Coroutines: Understanding the Compiler Transform"
//
// https://lewissbaker.github.io/2022/08/27/understanding-the-compiler-transform
// 
// By Lewis Baker
////////////////////////////////////////////////////////////////////////////////////////

#ifndef _HELPERS_H_
#define _HELPERS_H_

#include <cstddef>
#include <concepts>
#include <type_traits>
#include <memory>
#include <utility>

//////////////////////
// Helpers used by Coroutine Lowering

template<typename T>
struct manual_lifetime {
    manual_lifetime() noexcept = default;
    ~manual_lifetime() = default;

    // Not copyable/movable
    manual_lifetime(const manual_lifetime&) = delete;
    manual_lifetime(manual_lifetime&&) = delete;
    manual_lifetime& operator=(const manual_lifetime&) = delete;
    manual_lifetime& operator=(manual_lifetime&&) = delete;

    template<typename Factory>
        requires
    std::invocable<Factory&>&&
        std::same_as<std::invoke_result_t<Factory&>, T>
        T& construct_from(Factory factory) noexcept(std::is_nothrow_invocable_v<Factory&>) {
        return *::new (static_cast<void*>(&storage)) T(factory());
    }

    void destroy() noexcept(std::is_nothrow_destructible_v<T>) {
        std::destroy_at(std::launder(reinterpret_cast<T*>(&storage)));
    }

    T& get() & noexcept {
        return *std::launder(reinterpret_cast<T*>(&storage));
    }

private:
    alignas(T) std::byte storage[sizeof(T)];
};

template<typename T>
struct destructor_guard {
    explicit destructor_guard(manual_lifetime<T>& obj) noexcept
        : ptr_(std::addressof(obj))
    {}

    // non-movable
    destructor_guard(destructor_guard&&) = delete;
    destructor_guard& operator=(destructor_guard&&) = delete;

    ~destructor_guard() noexcept(std::is_nothrow_destructible_v<T>) {
        if (ptr_ != nullptr) {
            ptr_->destroy();
        }
    }

    void cancel() noexcept { ptr_ = nullptr; }

private:
    manual_lifetime<T>* ptr_;
};

// Parital specialisation for types that don't need their destructors called.
template<typename T>
    requires std::is_trivially_destructible_v<T>
struct destructor_guard<T> {
    explicit destructor_guard(manual_lifetime<T>&) noexcept {}
    void cancel() noexcept {}
};

// Class-template argument deduction to simplify usage
template<typename T>
destructor_guard(manual_lifetime<T>& obj) -> destructor_guard<T>;

template<typename Promise, typename... Params>
Promise construct_promise([[maybe_unused]] Params&... params) {
    if constexpr (std::constructible_from<Promise, Params&...>) {
        return Promise(params...);
    }
    else {
        return Promise();
    }
}

#endif
