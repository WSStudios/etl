
#include <functional>
#include <iostream>
#include <cstddef>

void foo(int a, float b, int c) {
    std::cout << a << ", " << b << ", " << c << std::endl;
}

template<typename T>
T getColumn(int index) {
    return T(index);
}

template<size_t... indexes>
struct index_tuple {};

template<size_t head, size_t... indexes>
struct index_tuple<head, indexes...> {
    typedef typename index_tuple<head-1, head-1, indexes...>::type type;
};

template<size_t... indexes>
struct index_tuple<0, indexes...> {
    typedef index_tuple<indexes...> type;
};

template<typename... Args>
struct make_index_tuple {
    typedef typename index_tuple<sizeof...(Args)>::type type;
};

template<typename... ColumnTypes, size_t... indexes>
void execute(const std::function<void(ColumnTypes...)> &callback, index_tuple<indexes...>) {
    // this should be done for every row in your query result
    callback(getColumn<ColumnTypes>(indexes)...);
}

template<typename... ColumnTypes>
void execute(const std::function<void(ColumnTypes...)> &callback) {
    execute(
        callback, 
        typename make_index_tuple<ColumnTypes...>::type()
    );
}

int main() {
    std::function<void(int, float, int)> fun(foo);
    execute(fun);
}