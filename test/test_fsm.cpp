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

#undef ETL_THROW_EXCEPTIONS

//#include "etl/platform.h"
#include "etl/exception.h"
#include "etl/pool.h"

template<typename E>
constexpr auto to_integral(E e) -> typename std::underlying_type<E>::type
{
   return static_cast<typename std::underlying_type<E>::type>(e);
}

namespace new_etl
{
	// template <typename... Ts>
	// struct largest_type;

	// template <typename T>
	// struct largest_type<T>
	// {
	// 	using type = T;
	// };

	// template <typename T, typename U, typename... Ts>
	// struct largest_type<T, U, Ts...>
	// {
	// 	using type = typename largest_type<typename std::conditional<
	// 										   (sizeof(U) <= sizeof(T)), T, U
	// 										   >::type, Ts...
	// 										   >::type;
	// };

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
}


namespace etl
{

	// -----------------------------------------
	template <typename THandler>
	class message
	{
	public:
		using handler_type = THandler;
		message() = default;
	};

	// -----------------------------------------
	template<typename TBase, typename TMessageType>
	struct message_handler
	{
		//static_assert(new_etl::is_static_castable<message_handler<TBase,TMessageType>*,TBase*>::value, "bad message_handler!");

		message_handler()
		{
			static_assert(new_etl::is_static_castable<decltype(this),TBase*>::value, "bad message_handler!");
		}

		//virtual TBase * on_event(const TMessageType&) = 0;
		virtual TBase * on_event(const TMessageType&)
		{
			return static_cast<TBase*>(this)->on_event_unknown();
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
		template <typename TMessage>
		void receive(const TMessage& message)
		{
			using handler_type = typename TMessage::handler_type;
			TStateType * next_state = static_cast<handler_type*>(p_state)->on_event(message);
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
	template<typename TFsm, typename TBaseStateType, typename... TState0>
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
		etl::generic_pool<new_etl::MaxSizeof<TState0...>::value, 16, 2> pool;
	};

	// -----------------------------------------
	/// Interface class for FSM states.
	// -----------------------------------------
	template<typename TInterface>
	class ifsm_state
	{
		//using MyType = ;
	public:
		virtual TInterface * on_enter_state() = 0;
		virtual void on_exit_state() = 0;
		//virtual ifsm_state * get_state_id() const = 0;
	};

	// -----------------------------------------
	/// FSM State
	// -----------------------------------------
	template <
		class TFsm,
		class TBaseType,
		typename... MessageType>
	class fsm_state
		: public ifsm_state<fsm_state<TFsm, TBaseType, MessageType...>>
		, public message_handler<TBaseType, MessageType>...
	{
	public:
		using TBase = ifsm_state<fsm_state<TFsm, MessageType...>>;

		fsm_state(TFsm& context)
			: p_context(&context)
		{}

		virtual ~fsm_state() = default;

		inline TFsm& get_fsm_context() const
		{
			return *p_context;
		}

		// TBaseType * on_enter_state() override { return this; } // By default, do nothing.
		void on_exit_state() override {}  // By default, do nothing.
		TBaseType* on_event_unknown()
		{
			printf("unknown event!\n");
			return static_cast<TBaseType*>(this);
		}

		virtual const char * description() const = 0;

	private:
		TFsm* p_context;

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
	class MyEvent0 : public etl::message<etl::message_handler<MyStateBase, MyEvent0>> {};
	class MyEvent1 : public etl::message<etl::message_handler<MyStateBase, MyEvent1>> {};

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

	using MyStateFactory =
		etl::StateFactory<MyFsm,
						  MyStateBase,
						  MyState0,
						  MyState1>;

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
		printf("MyState0::on_event MyEvent0.\n");
		return get_fsm_context().get_factory().create<MyState1>(get_fsm_context());
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
	MyStateBase * MyState1::on_event(const MyEvent1&)
	{
		printf("MyState1::on_event MyEvent1.\n");
		return get_fsm_context().get_factory().create<MyState0>(get_fsm_context());
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
				try
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
						fsm.receive(m);
					}

					printf("In State: %s\n", fsm.get_state().description());
				}
				catch (etl::fsm_exception e)
				{
					printf("exception: %s %s:%d\n", e.what(), e.file_name(), e.line_number());
				}
				catch (...)
				{
					printf("caught unknown exception!!\n");

				}

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


// namespace test
// {

// 	// -----------------------------------------
// 	// Events
// 	enum class EventId
// 	{
// 		START,
// 		STOP,
// 		STOPPED,
// 		SET_SPEED,
// 		RECURSIVE,
// 		UNSUPPORTED
// 	};

// 	// -----------------------------------------
// 	class Start : public etl::message<etl::message_handler<void, Start>>
// 	{
// 	};

// 	// -----------------------------------------
// 	class Stop : public etl::message<etl::message_handler<void, Stop>>
// 	{
// 	public:

// 		Stop() : isEmergencyStop(false) {}
// 		Stop(bool emergency) : isEmergencyStop(emergency) {}

// 		const bool isEmergencyStop;
// 	};

// 	// -----------------------------------------
// 	class SetSpeed : public etl::message<etl::message_handler<void, SetSpeed>>
// 	{
// 	public:

// 		SetSpeed(int speed_) : speed(speed_) {}

// 		const int speed;
// 	};

// 	// -----------------------------------------
// 	class Stopped : public etl::message<etl::message_handler<void, Stopped>>
// 	{
// 	};

// 	// -----------------------------------------
// 	class Recursive : public etl::message<etl::message_handler<void, Recursive>>
// 	{
// 	};

// 	// -----------------------------------------
// 	class Unsupported : public etl::message<etl::message_handler<void, Unsupported>>
// 	{
// 	};


// 	class MotorControlFsm;
// 	using MotorControlState = etl::fsm_state<
// 		MotorControlFsm,
// 		Start,
// 		Stop,
// 		Stopped,
// 		SetSpeed,
// 		Recursive,
// 		Unsupported>;

// 	// -----------------------------------------
// 	// -----------------------------------------
// 	// -----------------------------------------
// 	// STATES DECLARATIONS
// 	// -----------------------------------------
// 	// -----------------------------------------
// 	// -----------------------------------------

// 	using StateBase = MotorControlState::TBase;

// 	class Idle : public MotorControlState
// 	{
// 	public:
// 		Idle(MotorControlFsm& fsm) : MotorControlState(fsm) {}
// 		virtual MotorControlState * on_enter_state() override;
// 		virtual MotorControlState * on_event(const Start&) override;
// 	};

// 	class Running : public MotorControlState
// 	{
// 	public:
// 		Running(MotorControlFsm& fsm) : MotorControlState(fsm) {}
// 		MotorControlState * on_enter_state() override;
// 		MotorControlState * on_event(const Stop& event) override;
// 		MotorControlState * on_event(const SetSpeed& event) override;
// 		void on_exit_state() override;
// 	};

// 	class WindingDown : public MotorControlState
// 	{
// 	public:
// 		WindingDown(MotorControlFsm& fsm, int windDownTime) : MotorControlState(fsm), WindDownTime(windDownTime) {}
// 		MotorControlState * on_event(const Stopped&) override;
// 	private:
// 		int WindDownTime;
// 	};

// 	class Locked : public MotorControlState
// 	{
// 	public:
// 		Locked(MotorControlFsm& fsm) : MotorControlState(fsm) {}
// 	};

// 	class Error : public MotorControlState
// 	{
// 	public:
// 		Error(MotorControlFsm& fsm) : MotorControlState(fsm) {}
// 		MotorControlState * on_enter_state() override;
// 	};



// 	//using StateArray = std::array<MotorControlState*, 5>;
// 	using MotorStateFactory = etl::StateFactory<MotorControlFsm,
// 												MotorControlState,
// 												Idle,
// 												Running,
// 												WindingDown,
// 												Locked,
// 												Error>;

// 	//***********************************
// 	// The motor control FSM.
// 	//***********************************
// 	class MotorControlFsm
// 		: public etl::fsm<MotorStateFactory>
// 	{
// 		using Super = etl::fsm<MotorStateFactory>;

// 	public:

// 		const static int kRunningSpeed = 10;

// 		MotorControlFsm(MotorStateFactory& factory)
// 			: Super(factory, factory.emplace<Idle>(*this))
// 			, isLampOn(false)
// 			, desiredSpeed(0)
// 			, speed(0)
// 		{ }

// 		//***********************************
// 		void SetSpeedValue(int speed_)
// 		{
// 			desiredSpeed = speed_;
// 			speed = speed_;
// 		}

// 		//***********************************
// 		void SetRunning(bool on)
// 		{
// 			isLampOn = on;
// 			SetSpeedValue(on ? kRunningSpeed : 0);
// 		}

// 		bool isLampOn;
// 		int desiredSpeed;
// 		int speed;
// 	};


// 	// -----------------------------------------
// 	// -----------------------------------------
// 	// -----------------------------------------
// 	// STATES DEFINITIONS
// 	// -----------------------------------------
// 	// -----------------------------------------
// 	// -----------------------------------------


// 	//***********************************
// 	// The idle state.
// 	//***********************************
// 	MotorControlState * Idle::on_enter_state()
// 	{
// 		return this;
// 	}

// 	MotorControlState * Idle::on_event(const Start&)
// 	{
// 		return get_fsm_context().get_factory().emplace<Running>(get_fsm_context());
// 	}

// 	//***********************************
// 	// The running state.
// 	//***********************************
// 	MotorControlState * Running::on_enter_state()
// 	{
// 		get_fsm_context().SetRunning(true);
// 		return this;
// 	}

// 	//***********************************
// 	MotorControlState * Running::on_event(const Stop& event)
// 	{
// 		if (event.isEmergencyStop)
// 		{
// 			return get_fsm_context().get_factory().emplace<Idle>(get_fsm_context());
// 		}
// 		else
// 		{
// 			return get_fsm_context().get_factory().emplace<WindingDown>(get_fsm_context(), 2.0f);
// 		}
// 	}

// 	//***********************************
// 	MotorControlState * Running::on_event(const SetSpeed& event)
// 	{
// 		get_fsm_context().SetSpeedValue(event.speed);
// 		return this;
// 	}

// 	//***********************************
// 	void Running::on_exit_state()
// 	{
// 		get_fsm_context().SetRunning(false);
// 	}

// 	//***********************************
// 	// The winding down state.
// 	//***********************************
// 	MotorControlState * WindingDown::on_event(const Stopped&)
// 	{
// 		return get_fsm_context().get_factory().emplace<Idle>(get_fsm_context());
// 	}

// 	//***********************************
// 	// Error state
// 	//***********************************
// 	MotorControlState * Error::on_enter_state()
// 	{
// 		printf("Error state entered!!\n");
// 		return this;
// 	}



// 	SUITE(test_map)
// 	{
// 		//*************************************************************************
// 		TEST(test_fsm)
// 		{

// 			// Test starting
// 			{
// 				// Start the FSM.
// 				MotorStateFactory Factory;
// 				MotorControlFsm fsm(Factory);
// 				//Idle* idle = Factory.emplace<Idle>(fsm);
// 				//WindingDown* windingDown = Factory.emplace<WindingDown>(fsm, 5);

// 				Start m;
// 				fsm.receive(m);


// 				// Now in Idle state.
// 				//CHECK_EQUAL(to_integral(StateId::IDLE), to_integral(fsm.get_state_id()));
// 				//CHECK_EQUAL(to_integral(StateId::IDLE), to_integral(fsm.get_state().get_state_id()));

// 				CHECK_EQUAL(false, fsm.isLampOn);
// 				CHECK_EQUAL(0, fsm.speed);
// 			}

// 			// Test unhandled events.
// 			// {
// 			// 	// Start the FSM.
// 			// 	Fsm fsm(stateList);

// 			// 	fsm.receive(Stop());

// 			// 	CHECK_EQUAL(to_integral(StateId::ERROR), to_integral(fsm.get_state().get_state_id()));

// 			// 	CHECK_EQUAL(false, fsm.isLampOn);
// 			// 	CHECK_EQUAL(0, fsm.speed);
// 			// }

// 			// // test starting
// 			// {
// 			// 	// Start the FSM.
// 			// 	Fsm motorControl(stateList);

// 			// 	// Send Start event.
// 			// 	motorControl.receive(Start());

// 			// 	// Now in Running state.
// 			// 	CHECK_EQUAL(to_integral(StateId::RUNNING), to_integral(motorControl.get_state().get_state_id()));

// 			// 	CHECK_EQUAL(true, motorControl.isLampOn);
// 			// 	// CHECK_EQUAL(0, motorControl.speed);
// 			// }

// 			// // test starting, setting speed
// 			// {
// 			// 	// Start the FSM.
// 			// 	Fsm motorControl(stateList);

// 			// 	// Send Start event.
// 			// 	motorControl.receive(Start());

// 			// 	int speed = 2*Fsm::kRunningSpeed;
// 			// 	motorControl.receive(SetSpeed(speed));

// 			// 	// Now in Running state.
// 			// 	CHECK_EQUAL(to_integral(StateId::RUNNING), to_integral(motorControl.get_state().get_state_id()));

// 			// 	CHECK_EQUAL(true, motorControl.isLampOn);
// 			// 	CHECK_EQUAL(speed, motorControl.speed);
// 			// }

// 			// // test starting, stopping
// 			// {
// 			// 	// Start the FSM.
// 			// 	Fsm motorControl(stateList);

// 			// 	// Send Start event.
// 			// 	motorControl.receive(Start());
// 			// 	motorControl.receive(Stop());

// 			// 	// Now in WindingDown state.
// 			// 	CHECK_EQUAL(to_integral(StateId::WINDING_DOWN), to_integral(motorControl.get_state().get_state_id()));

// 			// 	CHECK_EQUAL(false, motorControl.isLampOn);
// 			// 	CHECK_EQUAL(0, motorControl.speed);
// 			// }
// 		}
// 	};
// }
