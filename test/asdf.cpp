
#include <stdio.h>

class imessage
{
public:
    imessage(int id_) : id(id_) {}
    int id;
};

template <int Type_>
class messageBase : public imessage
{
public:
    messageBase() : imessage(Type_) {}
    enum { Type = Type_ };
};

template <typename TDerived, typename T1, typename T2, typename T3, typename T4>
class MyClass
{
public:
    int Dispatch(const imessage& message)
    {
        int ret = 0;
        TDerived& derived = *static_cast<TDerived*>(this);
        switch (message.id)
        {
            case T1::Type: ret = derived.HandleMessage(static_cast<const T1&>(message)); break;
            case T2::Type: ret = derived.HandleMessage(static_cast<const T2&>(message)); break;
            case T3::Type: ret = derived.HandleMessage(static_cast<const T3&>(message)); break;
            case T4::Type: ret = derived.HandleMessage(static_cast<const T4&>(message)); break;
        }
        return ret;
    }
};

class message1 : public messageBase<1> {};
class message2 : public messageBase<2> {};
class message3 : public messageBase<3> {};
class message4 : public messageBase<4> {};

class DerivedClass : public MyClass<DerivedClass, message1, message2, message3, message4>
{
public:
    int HandleMessage(const message1& m) { return 1; }
    int HandleMessage(const message2& m) { return 2; }
    int HandleMessage(const message3& m) { return 3; }
    int HandleMessage(const message4& m) { return 4; }
};

int main (int argc, const char * argv[])
{
    DerivedClass test;
    message2 m;
    printf("message2: %d\n", test.Dispatch(m));
    return 0;
}