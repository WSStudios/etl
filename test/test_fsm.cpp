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

#include <utility>
#include <array>

#include "UnitTest++.h"

#undef ETL_THROW_EXCEPTIONS

#include "etl/platform.h"
#include "etl/array.h"
#include "etl/nullptr.h"
#include "etl/error_handler.h"
#include "etl/exception.h"
#include "etl/user_type.h"
//#include "etl/message_router.h"
#include "etl/integral_limits.h"
#include "etl/largest.h"
#include "etl/enum_type.h"
#include "etl/container.h"
#include "etl/packet.h"
#include "etl/queue.h"

//#include <iostream>

template<typename E>
constexpr auto to_integral(E e) -> typename std::underlying_type<E>::type 
{
   return static_cast<typename std::underlying_type<E>::type>(e);
}

template <typename... Ts>
struct largest_type;

template <typename T>
struct largest_type<T>
{
	using type = T;
};

template <typename T, typename U, typename... Ts>
struct largest_type<T, U, Ts...>
{
	using type = typename largest_type<typename std::conditional<(sizeof(U) <= sizeof(T)), T, U>::type, Ts...>::type;
};


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

	/// Allow alternative type for state id.
	typedef int32_t fsm_state_id_t;

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
	template<
		typename TFsm,
		typename TStateType,
		typename TStateFactory>
	class fsm
	{
	public:
		//*******************************************
		fsm(TStateFactory& factory_, TStateType * pInitialState)
			: factory(factory_)
			, p_state(nullptr)
		{
			// start state
			transition(pInitialState);
		}

		//*******************************************
		void transition(TStateType* p_to_state)
		{
			// Have we actually changed states?
			for (TStateType * p_next_state = p_to_state; p_next_state != p_state; )
			{
				if (p_state != nullptr)
				{
					p_state->on_exit_state();
				}
				p_state = p_next_state;
				ETL_ASSERT((p_state != nullptr), ETL_ERROR(etl::fsm_null_state_exception));

				p_next_state = p_state->on_enter_state();
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


	private:

		TStateFactory factory;
		TStateType * p_state;
	};

	template<typename TMessageType>
	struct message_handler
	{
		//using MyType = message_handler<typename TMessageType>;
		virtual message_handler * on_event(const TMessageType&) { return this; }
	}; 

	// -----------------------------------------
	/// Interface class for FSM states.
	// -----------------------------------------
	template <typename... MessageType>
	class ifsm_state : public message_handler<MessageType>...
	{
		//using MyType = ;
	public:
		virtual ifsm_state * on_enter_state() = 0;
		virtual void on_exit_state() = 0;
		virtual ifsm_state * get_state_id() const = 0;
	};

	template <
		typename TFsm, 
		typename TInterface>
	class fsm_state : public TInterface
	{
	public:
		fsm_state(TFsm& context)
			: p_context(&context)
		{}

		virtual ~fsm_state() = default;

		inline TFsm& get_fsm_context() const
		{
			return *p_context;
		}

	private:

		TInterface * on_enter_state() override { return this; } // By default, do nothing.
		void on_exit_state() override {}  // By default, do nothing.

		void set_fsm_context(TFsm& context)
		{
			p_context = &context;
		}

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
	// Events
	enum class EventId
	{
		START,
		STOP,
		STOPPED,
		SET_SPEED,
		RECURSIVE,
		UNSUPPORTED
	};

	class BaseState;

	// -----------------------------------------
	class Start : public etl::message<etl::message_handler<Start>>
	{
	};
	
	// -----------------------------------------
	class Stop : public etl::message<etl::message_handler<Stop>>
	{
	public:

		Stop() : isEmergencyStop(false) {}
		Stop(bool emergency) : isEmergencyStop(emergency) {}

		const bool isEmergencyStop;
	};

	// -----------------------------------------
	class SetSpeed : public etl::message<etl::message_handler<SetSpeed>>
	{
	public:

		SetSpeed(int speed_) : speed(speed_) {}

		const int speed;
	};

	// -----------------------------------------
	class Stopped : public etl::message<etl::message_handler<Stopped>>
	{
	};

	// -----------------------------------------
	class Recursive : public etl::message<etl::message_handler<Recursive>>
	{
	};

	// -----------------------------------------
	class Unsupported : public etl::message<etl::message_handler<Unsupported>>
	{
	};
 

	using MotorControlStateBase = 
		etl::ifsm_state<
			Start,
			Stop,
			Stopped,
			SetSpeed,
			Recursive,
			Unsupported>;

	//***********************************
	// The idle state.
	//***********************************
	class Idle : public MotorControlStateBase
	{
	public:
		//***********************************
		ifsm_state * on_enter_state() override
		{
			return this;
		}

		//***********************************
		ifsm_state * on_event(const Start&) override
		{
			return new Running();
		}
	};

	//***********************************
	// The running state.
	//***********************************
	class Running : public MotorControlStateBase
	{
	public:

		//***********************************
		ifsm_state * on_enter_state() override
		{
			get_fsm_context().SetRunning(true);
			return get_state_id();
		}

		//***********************************
		ifsm_state * on_event(const Stop& event) override
		{
			if (event.isEmergencyStop)
			{
				return new Idle();
			}
			else
			{
				return new WindowDown();
			}
		}

		//***********************************
		ifsm_state * on_event(const SetSpeed& event) override
		{
			get_fsm_context().SetSpeedValue(event.speed);
			return this;
		}

		//***********************************
		void on_exit_state() override
		{
			get_fsm_context().SetRunning(false);
		}
	};

	//***********************************
	// The winding down state.
	//***********************************
	class WindingDown : public MotorControlStateBase
	{
		using Super = StateBase;
	public:

		WindingDown(Fsm& fsm, int windDownTime) : Super(fsm), WindDownTime(windDownTime) {}

		//***********************************
		ifsm_state * on_event(const Stopped&) override
		{
			return new Idle();
		}

	private:
		int WindDownTime;

	};

	//***********************************
	// The locked state.
	//***********************************
	class Locked : public MotorControlStateBase
	{
	public:
	};

	class Error : public MotorControlStateBase
	{
	public:
		ifsm_state * on_enter_state() override
		{
			printf("Error state entered!!\n");
			return this;
		}
	};


	template<typename TFsm, typename TStateBase, typename... TState0>
	class StateFactory
	{
		using largest = largest_type<TState0...>;
		union 
		{
			char buffer[sizeof(largest)];
			TStateBase * State;
		};
	public:
	
		//StateFactory(TFsm& fsm) : Fsm(fsm) {}

		// template <typename _ValueType, typename... _Args>
		// typename __any_constructible<_Decay<_ValueType>&, _Decay<_ValueType>, _Args&&...>::type emplace(_Args&&... __args)
		// {
		// 	__do_emplace<_Decay<_ValueType>>(std::forward<_Args>(__args)...);
		// 	any::_Arg __arg;
		// 	this->_M_manager(any::_Op_access, this, &__arg);
		// 	return *static_cast<_Decay<_ValueType>*>(__arg._M_obj);
		// }

		template<typename ValueType, typename... _Args>
		ValueType& emplace(TFsm& fsm, _Args&&... args)
		{
			::new (buffer) ValueType(fsm, std::forward<_Args>(args)...);
			return *static_cast<ValueType*>(State);
		}
	};

	//using StateArray = std::array<MotorControlStateBase*, 5>;
	class MotorControlFsm;
	using MotorStateFactory = StateFactory<MotorControlFsm, MotorControlStateBase>;

	//***********************************
	// The motor control FSM.
	//***********************************
	class MotorControlFsm 
		: public etl::fsm<
			MotorControlFsm,
			MotorControlStateBase,
			MotorStateFactory>
	{
	public:
		using Super = etl::fsm<MotorControlFsm, MotorControlStateBase, MotorStateFactory>;

		const static int kRunningSpeed = 10;

		MotorControlFsm(MotorStateFactory& factory)
			: Super(factory, new Idle())
			, isLampOn(false)
			, desiredSpeed(0)
			, speed(0)
		{ }

		//***********************************
		void SetSpeedValue(int speed_)
		{
			desiredSpeed = speed_;
			speed = speed_;
		}

		//***********************************
		void SetRunning(bool on)
		{
			isLampOn = on;
			SetSpeedValue(on ? kRunningSpeed : 0);
		}

		bool isLampOn;
		int desiredSpeed;
		int speed;
	};

	using Fsm = MotorControlFsm;
	//using StateBase = etl::fsm_state<Fsm, MotorControlStateBase, Idle, Running, WindingDown, Locked, Error>;

	// template<typename TFsm, typename TStateBase, typename TStateId>
	// class IStateFactory
	// {
	// public:
	// }


	// The states.

	SUITE(test_map)
	{
		//*************************************************************************
		TEST(test_fsm)
		{

			// Test starting
			{
				// Start the FSM.
				Fsm fsm(stateList);
				StateFactory<MotorControlStateBase, Idle, Running, WindingDown, Locked, Error> Factory(fsm);
				Idle& idle = Factory.emplace<Idle>();
				WindingDown& windingDown = Factory.emplace<WindingDown>(5);

				// Now in Idle state.
				//CHECK_EQUAL(to_integral(StateId::IDLE), to_integral(fsm.get_state_id()));
				CHECK_EQUAL(to_integral(StateId::IDLE), to_integral(fsm.get_state().get_state_id()));

				CHECK_EQUAL(false, fsm.isLampOn);
				CHECK_EQUAL(0, fsm.speed);
			}

			// Test unhandled events.
			{
				// Start the FSM.
				Fsm fsm(stateList);

				fsm.receive(Stop());

				CHECK_EQUAL(to_integral(StateId::ERROR), to_integral(fsm.get_state().get_state_id()));

				CHECK_EQUAL(false, fsm.isLampOn);
				CHECK_EQUAL(0, fsm.speed);
			}

			// test starting
			{
				// Start the FSM.
				Fsm motorControl(stateList);

				// Send Start event.
				motorControl.receive(Start());

				// Now in Running state.
				CHECK_EQUAL(to_integral(StateId::RUNNING), to_integral(motorControl.get_state().get_state_id()));

				CHECK_EQUAL(true, motorControl.isLampOn);
				// CHECK_EQUAL(0, motorControl.speed);
			}

			// test starting, setting speed
			{
				// Start the FSM.
				Fsm motorControl(stateList);

				// Send Start event.
				motorControl.receive(Start());

				int speed = 2*Fsm::kRunningSpeed;
				motorControl.receive(SetSpeed(speed));

				// Now in Running state.
				CHECK_EQUAL(to_integral(StateId::RUNNING), to_integral(motorControl.get_state().get_state_id()));

				CHECK_EQUAL(true, motorControl.isLampOn);
				CHECK_EQUAL(speed, motorControl.speed);
			}

			// test starting, stopping
			{
				// Start the FSM.
				Fsm motorControl(stateList);

				// Send Start event.
				motorControl.receive(Start());
				motorControl.receive(Stop());

				// Now in WindingDown state.
				CHECK_EQUAL(to_integral(StateId::WINDING_DOWN), to_integral(motorControl.get_state().get_state_id()));

				CHECK_EQUAL(false, motorControl.isLampOn);
				CHECK_EQUAL(0, motorControl.speed);
			}
		}
	};
}
