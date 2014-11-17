#ifndef ContainerTags_hpp
#define ContainerTags_hpp

#include <type_traits>

namespace ContainerTags{

namespace details{
    inline constexpr auto is_container_impl(...) -> std::false_type {
        return std::false_type{};
    }

    template <typename C>
    constexpr auto is_container_impl(C const* c) ->
        decltype(begin(*c), end(*c), std::true_type{})
    {
        return std::true_type{};
    }


    inline constexpr auto is_associative_container_impl(...)
        -> std::false_type
    { return std::false_type{}; }

    template <typename C, typename = typename C::key_type>
    constexpr auto is_associative_container_impl(C const*) -> std::true_type {
        return std::true_type{};
    }
};


    template <typename C>
    constexpr auto is_container(C const& c) -> decltype(details::is_container_impl(&c)) {
        return details::is_container_impl(&c);
    }

    template <typename C>
    constexpr auto is_associative(C const& c)
        -> decltype(details::is_associative_container_impl(&c))
    {
        return details::is_associative_container_impl(&c);
    }

    template <typename C>
    constexpr auto is_sequence(C const& c)
        -> decltype(details::is_associative_container_impl(&c))
    {
        return details::is_associative_container_impl(&c);
    }

};


#endif // AssociativeContainer_hpp

