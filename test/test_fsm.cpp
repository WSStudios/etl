// -----------------------------------------
// The MIT License(MIT)
//
// Embedded Template Library.
// https://github.com/ETLCPP/etl
// https://www.etlcpp.com
//
// Copyright(c) 2017 jwellbelove
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//  -----------------------------------------

#include <type_traits>

#include "UnitTest++.h"

#define ETL_THROW_EXCEPTIONS

#include "etl/platform.h"
#include "etl/exception.h"
#include "etl/error_handler.h"
#include "etl/pool.h"

template<typename E>
constexpr auto to_integral(E e) -> typename std::underlying_type<E>::type
{
   return static_cast<typename std::underlying_type<E>::type>(e);
}

namespace new_etl
{
	template <class F, class T, class = T>
	struct is_static_castable : std::false_type
	{};

	template <class F, class T>
	struct is_static_castable<F, T, decltype(static_cast<T>(std::declval<F>()))> : std::true_type
	{};

	template<typename... Ts>
	struct MaxSizeof
	{
		static constexpr size_t value = 0;
	};

	template<typename T, typename... Ts>
	struct MaxSizeof<T, Ts...>
	{
		static constexpr size_t value = std::max(sizeof(T), MaxSizeof<Ts...>::value);
	};


	template<typename... Ts>
	struct MaxAlignmentof
	{
		static constexpr size_t value = 0;
	};

	template<typename T, typename... Ts>
	struct MaxAlignmentof<T, Ts...>
	{
		static constexpr size_t value = std::max(alignof(T), MaxSizeof<Ts...>::value);
	};

	template<typename T, typename... Ts>
	struct MaxTraits
	{
		static constexpr size_t MaxSize = MaxSizeof<T, Ts...>::value;
		static constexpr size_t MaxAlignment = MaxAlignmentof<T, Ts...>::value;
	};

}

namespace etl
{

	// -----------------------------------------
	template <typename THandler>
	class event
	{
	public:
		using handler_type = THandler;
		event() = default;
		const char * description() const { return typeid(decltype(this)).name(); }
	};

	// -----------------------------------------
	template<typename TBase, typename TEventType>
	struct event_handler
	{
		event_handler()
		{
			static_assert(new_etl::is_static_castable<decltype(this),TBase*>::value, "bad event_handler!");
		}

		virtual TBase * on_event(const TEventType& event)
		{
			return static_cast<TBase*>(this)->on_event_unknown(event.description());
		}
	};



	// -----------------------------------------
	/// Base exception class for FSM.
	// -----------------------------------------
	class fsm_exception : public etl::exception
	{
	public:

		fsm_exception(string_type reason_, string_type file_name_, numeric_type line_number_)
			: etl::exception(reason_, file_name_, line_number_)
		{
		}
	};

	// -----------------------------------------
	/// Exception for null state pointer.
	// -----------------------------------------
	class fsm_null_state_exception : public etl::fsm_exception
	{
	public:

		fsm_null_state_exception(string_type file_name_, numeric_type line_number_)
			: etl::fsm_exception(ETL_ERROR_TEXT("fsm:null state", ETL_FILE"A"), file_name_, line_number_)
		{
		}
	};

	class fsm_unknown_event_exception : public etl::fsm_exception
	{
	public:

		fsm_unknown_event_exception(const char * event_description_)
			: etl::fsm_exception(ETL_ERROR_TEXT("fsm:unknown event", ETL_FILE"A"), "", 0)
			, event_description(event_description_)
		{
		}
	private:
		const char * event_description;
	};


	// -----------------------------------------
	/// Exception for invalid state id.
	// -----------------------------------------
	class fsm_state_id_exception : public etl::fsm_exception
	{
	public:

		fsm_state_id_exception(string_type file_name_, numeric_type line_number_)
			: etl::fsm_exception(ETL_ERROR_TEXT("fsm:state id", ETL_FILE"B"), file_name_, line_number_)
		{
		}
	};

	// -----------------------------------------
	/// Exception for incompatible state list.
	// -----------------------------------------
	class fsm_state_list_exception : public etl::fsm_exception
	{
	public:

		fsm_state_list_exception(string_type file_name_, numeric_type line_number_)
			: etl::fsm_exception(ETL_ERROR_TEXT("fsm:state list", ETL_FILE"C"), file_name_, line_number_)
		{
		}
	};



	// -----------------------------------------
	/// The FSM class.
	// -----------------------------------------
	template<typename TStateFactory, typename TStateType>
	class fsm
	{
		using StateType = TStateType;
	public:
		//*******************************************
		fsm(TStateFactory* factory_)
			: factory(factory_)
			, p_state(nullptr)
		{}

		// start state
		void start(TStateType * pInitialState)
		{
			transition(pInitialState);
		}

		//*******************************************
		void transition(TStateType* p_to_state)
		{
			ETL_ASSERT((p_to_state != nullptr), ETL_ERROR(etl::fsm_null_state_exception));

			// Transition until we don't change states
			for (TStateType * p_next_state = p_to_state; p_next_state != p_state; )
			{
				if (p_state != nullptr)
				{
					p_state->on_exit_state();
					factory->destroy(p_state);
				}
				p_state = p_next_state;
				ETL_ASSERT((p_state != nullptr), ETL_ERROR(etl::fsm_null_state_exception));

				p_next_state = static_cast<TStateType*>(p_state->on_enter_state());
			}
		}

		//*******************************************
		template <typename TEvent>
		void receive(const TEvent& event)
		{
			using handler_type = typename TEvent::handler_type;
			TStateType * next_state = static_cast<handler_type*>(p_state)->on_event(event);
			transition(next_state);
		}

		//*******************************************
		const TStateType& get_state() const
		{
			ETL_ASSERT(p_state != nullptr, ETL_ERROR(etl::fsm_null_state_exception));
			return *p_state;
		}

		TStateFactory& get_factory() { return *factory; }

	private:

		TStateFactory* factory;
		TStateType* p_state;
	};

	// -----------------------------------------
	/// State Factory Class
	// -----------------------------------------
	template<
		typename TFsm,
		typename TBaseStateType,
		size_t MAX_SIZE,
		size_t MAX_ALIGNMENT>
	class StateFactory
	{
	public:
		template<typename ValueType, typename... _Args>
		ValueType* create(TFsm& fsm, _Args&&... args)
		{
			return new (pool.template allocate<ValueType>()) ValueType(fsm, std::forward<_Args>(args)...);
		}
		template<typename ValueType>
		void destroy(ValueType* item)
		{
			item->~ValueType();
			pool.release(item);
		}
	private:
		etl::generic_pool<MAX_SIZE, MAX_ALIGNMENT, 2> pool;
	};

	// -----------------------------------------
	/// Interface class for FSM states.
	// -----------------------------------------
	template<typename TInterface>
	class ifsm_state
	{
	public:
		virtual TInterface * on_enter_state() = 0;
		virtual void on_exit_state() = 0;
	};

	// -----------------------------------------
	/// FSM State
	// -----------------------------------------
	template <
		class TFsm,
		class TBaseType,
		typename... EventType>
	class fsm_state
		: public ifsm_state<fsm_state<TFsm, TBaseType, EventType...>>
		, public event_handler<TBaseType, EventType>...
	{
	public:
		using TBase = ifsm_state<fsm_state<TFsm, EventType...>>;

		fsm_state(TFsm& fsm)
			: p_fsm(&fsm)
		{}

		virtual ~fsm_state() = default;

		inline TFsm& get_fsm() const
		{
			return *p_fsm;
		}

		// TBaseType * on_enter_state() override { return this; } // By default, do nothing.
		void on_exit_state() override {}  // By default, do nothing.
		TBaseType* on_event_unknown(const char * desc)
		{
			throw etl::fsm_unknown_event_exception(desc);
			return static_cast<TBaseType*>(this);
		}

		virtual const char * description() const = 0;

	private:
		TFsm* p_fsm;

		// Disabled.
		fsm_state(const fsm_state&);
		fsm_state& operator =(const fsm_state&);
	};


}

// -----------------------------------------
// -----------------------------------------
// -----------------------------------------
// -----------------------------------------

namespace test
{

	// -----------------------------------------
	class MyStateBase;
	class MyEvent0 : public etl::event<etl::event_handler<MyStateBase, MyEvent0>> {};
	class MyEvent1 : public etl::event<etl::event_handler<MyStateBase, MyEvent1>> {};

	class MyFsm;
	class MyStateBase;
	using FsmBase = etl::fsm_state<MyFsm, MyStateBase, MyEvent0, MyEvent1>;
	class MyStateBase : public FsmBase
	{
		using Super = FsmBase;
	public:
		MyStateBase(MyFsm& fsm) : Super(fsm) {}
		virtual MyStateBase * on_enter_state() override { printf("oops!!\n"); return this; }
		virtual const char * description() const override { return "MyStateBase"; }
	};

	// -----------------------------------------
	// -----------------------------------------
	// -----------------------------------------
	// STATES DECLARATIONS
	// -----------------------------------------
	// -----------------------------------------
	// -----------------------------------------

	// -----------------------------------------
	class MyState0 : public MyStateBase
	{
	public:
		MyState0(MyFsm& fsm);
		virtual MyStateBase * on_enter_state() override;
		virtual MyStateBase * on_event(const MyEvent0&) override;
		virtual const char * description() const override { return "MyState0"; }
		void dump0() const;
	private:
		int numbers[16];
	};

	// -----------------------------------------
	class MyState1 : public MyStateBase
	{
	public:
		MyState1(MyFsm& fsm);
		virtual MyStateBase * on_enter_state() override;
		virtual MyStateBase * on_event(const MyEvent1&) override;
		virtual const char * description() const override { return "MyState1"; }
		// void dump1() const;
	private:
		//int numbers[16];
	};

	// -----------------------------------------

	typedef new_etl::MaxTraits<MyState0, MyState1> StateTraits;

	using MyStateFactory =
		etl::StateFactory<MyFsm,
						  MyStateBase,
						  StateTraits::MaxSize,
						  StateTraits::MaxAlignment>;

	using MyFsmBase = etl::fsm<MyStateFactory, MyStateBase>;

	// -----------------------------------------
	// The Test FSM.
	// -----------------------------------------
	class MyFsm : public MyFsmBase
	{
		using Super = MyFsmBase;

	public:

		MyFsm(MyStateFactory* factory)
			: Super(factory)
		{ }
	};


	// -----------------------------------------
	// -----------------------------------------
	// -----------------------------------------
	// STATES DEFINITIONS
	// -----------------------------------------
	// -----------------------------------------
	// -----------------------------------------

	// -----------------------------------------
	MyState0::MyState0(MyFsm& fsm)
		: MyStateBase(fsm)
	{
		for (int32_t ii = 0; ii < 16; ++ii)
		{
			numbers[ii] = ii;
		}
	}

	// -----------------------------------------
	MyStateBase * MyState0::on_enter_state()
	{
		printf("MyState0::on_enter_state.\n");
		return this;
	}

	// -----------------------------------------
	MyStateBase * MyState0::on_event(const MyEvent0& event)
	{
		printf("%s::on_event %s\n", description(), event.description());
		return get_fsm().get_factory().create<MyState1>(get_fsm());
	}

	// -----------------------------------------
	void MyState0::dump0() const
	{
		for (int32_t ii = 0; ii < 16; ++ii)
		{
			printf("%d: %d\n", ii, numbers[ii]);
		}
	}

	// -----------------------------------------
	// -----------------------------------------

	// -----------------------------------------
	MyState1::MyState1(MyFsm& fsm)
		: MyStateBase(fsm)
	{
		// for (int32_t ii = 0; ii < 16; ++ii)
		// {
		// 	numbers[ii] = 16-ii;
		// }
	}

	// -----------------------------------------
	MyStateBase * MyState1::on_enter_state()
	{
		printf("MyState1::on_enter_state.\n");
		return this;
	}

	// -----------------------------------------
	MyStateBase * MyState1::on_event(const MyEvent1& event)
	{
		printf("%s::on_event %s\n", description(), event.description());
		return get_fsm().get_factory().create<MyState0>(get_fsm());
	}

	// // -----------------------------------------
	// void MyState1::dump1() const
	// {
	// 	for (int32_t ii = 0; ii < 16; ++ii)
	// 	{
	// 		printf("%d: %d\n", ii, numbers[ii]);
	// 	}
	// }





	// -----------------------------------------
	// -----------------------------------------
	// -----------------------------------------
	// -----------------------------------------
	// -----------------------------------------

	SUITE(test_map)
	{
		//*************************************************************************
		TEST(test_fsm)
		{

			// Test starting
			{
				//try
				{
					printf("Test started.\n");

					// Start the FSM.
					MyStateFactory Factory;
					MyFsm fsm(&Factory);

					fsm.start(Factory.create<MyState0>(fsm));

					printf("In State: %s\n", fsm.get_state().description());

					{
						const MyState0& s = static_cast<const MyState0&>(fsm.get_state());
						s.dump0();
					}

					{
						MyEvent0 m;
						fsm.receive(m);
					}

					printf("In State: %s\n", fsm.get_state().description());

					{
						MyEvent1 m;
						fsm.receive(m);
					}

					printf("In State: %s\n", fsm.get_state().description());

					{
						MyEvent1 m;
						CHECK_THROW(fsm.receive(m), etl::fsm_unknown_event_exception);
					}

					printf("In State: %s\n", fsm.get_state().description());
				}
				// catch (etl::fsm_exception e)
				// {
				// 	printf("exception: %s %s:%d\n", e.what(), e.file_name(), e.line_number());
				// }
				// catch (...)
				// {
				// 	printf("caught unknown exception!!\n");

				// }

				//Idle* idle = Factory.emplace<Idle>(fsm);
				//WindingDown* windingDown = Factory.emplace<WindingDown>(fsm, 5);

				// Start m;
				// fsm.receive(m);


				// // Now in Idle state.
				// //CHECK_EQUAL(to_integral(StateId::IDLE), to_integral(fsm.get_state_id()));
				// //CHECK_EQUAL(to_integral(StateId::IDLE), to_integral(fsm.get_state().get_state_id()));

				// CHECK_EQUAL(false, fsm.isLampOn);
				// CHECK_EQUAL(0, fsm.speed);
			}
		}
	};
}
