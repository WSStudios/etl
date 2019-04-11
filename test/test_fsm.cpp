/******************************************************************************
The MIT License(MIT)

Embedded Template Library.
https://github.com/ETLCPP/etl
https://www.etlcpp.com

Copyright(c) 2017 jwellbelove

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

#include <utility>

#include "UnitTest++.h"

#include "etl/platform.h"
#include "etl/array.h"
#include "etl/nullptr.h"
#include "etl/error_handler.h"
#include "etl/exception.h"
#include "etl/user_type.h"
#include "etl/message_router.h"
#include "etl/integral_limits.h"
#include "etl/largest.h"
#include "etl/enum_type.h"
#include "etl/container.h"
#include "etl/packet.h"
#include "etl/queue.h"

//#include <iostream>


namespace etl
{

  /// Allow alternative type for state id.
#if !defined(ETL_FSM_STATE_ID_TYPE)
    typedef uint_least8_t fsm_state_id_t;
#else
    typedef ETL_FSM_STATE_ID_TYPE fsm_state_id_t;
#endif

  // For internal FSM use.
  typedef typename etl::larger_type<etl::message_id_t>::type fsm_internal_id_t;

  //***************************************************************************
  /// Base exception class for FSM.
  //***************************************************************************
  class fsm_exception : public etl::exception
  {
  public:

    fsm_exception(string_type reason_, string_type file_name_, numeric_type line_number_)
      : etl::exception(reason_, file_name_, line_number_)
    {
    }
  };

  //***************************************************************************
  /// Exception for null state pointer.
  //***************************************************************************
  class fsm_null_state_exception : public etl::fsm_exception
  {
  public:

    fsm_null_state_exception(string_type file_name_, numeric_type line_number_)
      : etl::fsm_exception(ETL_ERROR_TEXT("fsm:null state", ETL_FILE"A"), file_name_, line_number_)
    {
    }
  };

  //***************************************************************************
  /// Exception for invalid state id.
  //***************************************************************************
  class fsm_state_id_exception : public etl::fsm_exception
  {
  public:

    fsm_state_id_exception(string_type file_name_, numeric_type line_number_)
      : etl::fsm_exception(ETL_ERROR_TEXT("fsm:state id", ETL_FILE"B"), file_name_, line_number_)
    {
    }
  };

  //***************************************************************************
  /// Exception for incompatible state list.
  //***************************************************************************
  class fsm_state_list_exception : public etl::fsm_exception
  {
  public:

    fsm_state_list_exception(string_type file_name_, numeric_type line_number_)
      : etl::fsm_exception(ETL_ERROR_TEXT("fsm:state list", ETL_FILE"C"), file_name_, line_number_)
    {
    }
  };



  //***************************************************************************
  /// The FSM class.
  //***************************************************************************
  template<typename TStateType>
  class fsm : public etl::imessage_router
  {
  public:
    using TBaseFsm = typename TStateType::MyTFsm;

    //*******************************************
    /// Constructor.
    //*******************************************
    fsm(etl::message_router_id_t id)
      : imessage_router(id),
        p_state(nullptr)
    { }

    //*******************************************
    /// Set the states for the FSM
    //*******************************************
    template <typename TSize>
    void set_states(TStateType** p_states, TSize size)
    {
      state_list       = p_states;
      number_of_states = etl::fsm_state_id_t(size);
      ETL_ASSERT((number_of_states > 0), ETL_ERROR(etl::fsm_state_list_exception));

      for (etl::fsm_state_id_t i = 0; i < size; ++i)
      {
        ETL_ASSERT((state_list[i] != nullptr), ETL_ERROR(etl::fsm_null_state_exception));
        state_list[i]->set_fsm_context(static_cast<TBaseFsm&>(*this));
      }
    }

    //*******************************************
    /// Starts the FSM.
    /// Can only be called once.
    /// Subsequent calls will do nothing.
    ///\param call_on_enter_state If will call on_enter_state() for the first state. Default = true.
    //*******************************************
    void start(etl::fsm_state_id_t initial_state_id)
    {
      ETL_ASSERT(initial_state_id >= 0 && initial_state_id < number_of_states, ETL_ERROR(etl::fsm_state_id_exception));
      p_state = nullptr;
      transition(state_list[initial_state_id]);
    }

    void transition(TStateType* p_to_state)
    {
      // Have we actually changed states?
      for (TStateType * p_next_state = p_to_state; 
           p_next_state != p_state;
           )
      {
        if (p_state != nullptr)
        {
          p_state->on_exit_state();
        }
        p_state = p_next_state;

        etl::fsm_state_id_t next_state_id = p_state->on_enter_state();
        ETL_ASSERT(next_state_id >= 0 && next_state_id < number_of_states, ETL_ERROR(etl::fsm_state_id_exception));
        p_next_state = state_list[next_state_id];
      }
    }

    //*******************************************
    /// Top level message handler for the FSM.
    //*******************************************
    template <typename T>
    void receive(const T& message)
    {
      static etl::null_message_router nmr;
      receive(nmr, message);
    }

    //*******************************************
    /// Top level message handler for the FSM.
    //*******************************************
    template <typename TMessage>
    void receive(etl::imessage_router& source, const TMessage& message)
    {
        etl::fsm_state_id_t next_state_id = p_state->on_event(source, message);
        ETL_ASSERT(next_state_id < number_of_states, ETL_ERROR(etl::fsm_state_id_exception));
        transition(state_list[next_state_id]);
    }

    using imessage_router::accepts;

    //*******************************************
    /// Does this FSM accept the message id?
    /// Yes, it accepts everything!
    //*******************************************
    bool accepts(etl::message_id_t) const
    {
      return true;
    }

    //*******************************************
    /// Gets the current state id.
    //*******************************************
    etl::fsm_state_id_t get_state_id() const
    {
      ETL_ASSERT(p_state != nullptr, ETL_ERROR(etl::fsm_null_state_exception));
      return p_state->get_state_id();
    }

    //*******************************************
    /// Gets a reference to the current state interface.
    //*******************************************
    TStateType& get_state()
    {
      ETL_ASSERT(p_state != nullptr, ETL_ERROR(etl::fsm_null_state_exception));
      return *p_state;
    }

    //*******************************************
    /// Gets a const reference to the current state interface.
    //*******************************************
    const TStateType& get_state() const
    {
      ETL_ASSERT(p_state != nullptr, ETL_ERROR(etl::fsm_null_state_exception));
      return *p_state;
    }

    //*******************************************
    /// Checks if the FSM has been started.
    //*******************************************
    bool is_started() const
    {
      return p_state != nullptr;
    }

    //*******************************************
    /// Reset the FSM to pre-started state.
    ///\param call_on_exit_state If true will call on_exit_state() for the current state. Default = false.
    //*******************************************
    void reset(bool call_on_exit_state = false)
    {
      if ((p_state != nullptr) && call_on_exit_state)
      {
        p_state->on_exit_state();
      }

      p_state = nullptr;
    }

  private:

    TStateType*    p_state;          ///< A pointer to the current state.
    TStateType**   state_list;       ///< The list of added states.
    etl::fsm_state_id_t number_of_states; ///< The number of states.
  };


  //***************************************************************************
  /// Interface class for FSM states.
  //***************************************************************************
  template <typename TFsm, typename TStateBase, typename... MessageType>
  class fsm_state : 
    public message_handler<etl::imessage, etl::fsm_state_id_t>, 
    public message_handler<MessageType, etl::fsm_state_id_t>...
  {
  public: 
    typedef TFsm MyTFsm;
    friend class fsm<TStateBase>;

    etl::fsm_state_id_t get_state_id() const
    {
      return state_id;
    }

  protected:

    fsm_state(etl::fsm_state_id_t state_id_)
      : state_id(state_id_),
        p_context(nullptr)
    {}

    virtual ~fsm_state() = default;
    inline TFsm& get_fsm_context() const
    {
      return *p_context;
    }

  private:

    virtual fsm_state_id_t on_enter_state() { return state_id; } // By default, do nothing.
    virtual void on_exit_state() {}  // By default, do nothing.

    void set_fsm_context(TFsm& context)
    {
      p_context = &context;
    }

    const etl::fsm_state_id_t state_id;
    TFsm* p_context;

    // Disabled.
    fsm_state(const fsm_state&);
    fsm_state& operator =(const fsm_state&);
  };


  }



namespace test
{
  const etl::message_router_id_t MOTOR_CONTROL = 0;


  //***************************************************************************
  // Events
  struct EventId
  {
    enum enum_type
    {
      START,
      STOP,
      STOPPED,
      SET_SPEED,
      RECURSIVE,
      UNSUPPORTED
    };

    ETL_DECLARE_ENUM_TYPE(EventId, etl::message_id_t)
    ETL_ENUM_TYPE(START,       "Start")
    ETL_ENUM_TYPE(STOP,        "Stop")
    ETL_ENUM_TYPE(STOPPED,     "Stopped")
    ETL_ENUM_TYPE(SET_SPEED,   "Set Speed")
    ETL_ENUM_TYPE(RECURSIVE,   "Recursive")
    ETL_ENUM_TYPE(UNSUPPORTED, "Unsupported")
    ETL_END_ENUM_TYPE
  };

  class BaseState;

  //***********************************
  class Start : public etl::message<EventId::START, etl::message_handler<Start, etl::fsm_state_id_t>>
  {
  };
  
  //***********************************
  class Stop : public etl::message<EventId::STOP, etl::message_handler<Stop, etl::fsm_state_id_t>>
  {
  public:

    Stop() : isEmergencyStop(false) {}
    Stop(bool emergency) : isEmergencyStop(emergency) {}

    const bool isEmergencyStop;
  };

  //***********************************
  class SetSpeed : public etl::message<EventId::SET_SPEED, etl::message_handler<SetSpeed, etl::fsm_state_id_t>>
  {
  public:

    SetSpeed(int speed_) : speed(speed_) {}

    const int speed;
  };

  //***********************************
  class Stopped : public etl::message<EventId::STOPPED, etl::message_handler<Stopped, etl::fsm_state_id_t>>
  {
  };

  //***********************************
  class Recursive : public etl::message<EventId::RECURSIVE, etl::message_handler<Recursive, etl::fsm_state_id_t>>
  {
  };

  //***********************************
  class Unsupported : public etl::message<EventId::UNSUPPORTED, etl::message_handler<Unsupported, etl::fsm_state_id_t>>
  {
  };
 

  //***************************************************************************
  // States
  struct StateId
  {
    enum enum_type
    {
      IDLE,
      RUNNING,
      WINDING_DOWN,
      LOCKED,
      NUMBER_OF_STATES
    };

    ETL_DECLARE_ENUM_TYPE(StateId, etl::fsm_state_id_t)
    ETL_ENUM_TYPE(IDLE,         "Idle")
    ETL_ENUM_TYPE(RUNNING,      "Running")
    ETL_ENUM_TYPE(WINDING_DOWN, "Winding Down")
    ETL_ENUM_TYPE(LOCKED,       "Locked")
    ETL_END_ENUM_TYPE
  };

  template <typename TFsm>
  class StateBase 
    : public etl::fsm_state<
      TFsm,
      StateBase<TFsm>,
      Start,
      Stop,
      Stopped,
      SetSpeed,
      Recursive,
      Unsupported
      >
  {
  public:
    using Super = etl::fsm_state<
      TFsm,
      StateBase<TFsm>,
      Start,
      Stop,
      Stopped,
      SetSpeed,
      Recursive,
      Unsupported
      >;

    using etl::message_handler<Start, etl::fsm_state_id_t>::on_event;
    using etl::message_handler<Stop, etl::fsm_state_id_t>::on_event;
    using etl::message_handler<Stopped, etl::fsm_state_id_t>::on_event;
    using etl::message_handler<SetSpeed, etl::fsm_state_id_t>::on_event;
    using etl::message_handler<Recursive, etl::fsm_state_id_t>::on_event;
    using etl::message_handler<Unsupported, etl::fsm_state_id_t>::on_event;
    using etl::message_handler<etl::imessage, etl::fsm_state_id_t>::on_event;

    StateBase(etl::fsm_state_id_t id) : Super(id) {} 
    //virtual const StateBase& get_receiver() const = 0;
  };

  //***********************************
  // The motor control FSM.
  //***********************************
  class MotorControl : public etl::fsm<StateBase<MotorControl>>
  {
  public:
    using TStateBase = StateBase<MotorControl>;

    MotorControl(TStateBase** p_states, size_t size)
      : fsm(MOTOR_CONTROL)
    {
      set_states(p_states, size);
      ClearStatistics();
    }

    //***********************************
    void ClearStatistics()
    {
      startCount = 0;
      stopCount = 0;
      setSpeedCount = 0;
      unknownCount = 0;
      stoppedCount = 0;
      isLampOn = false;
      speed = 0;
    }

    //***********************************
    void SetSpeedValue(int speed_)
    {
      speed = speed_;
    }

    //***********************************
    void TurnRunningLampOn()
    {
      isLampOn = true;
    }

    //***********************************
    void TurnRunningLampOff()
    {
      isLampOn = false;
    }

    //***********************************
    template <typename T>
    void queue_recursive_message(const T& message)
    {
      messageQueue.emplace(message);
    }

    typedef etl::largest<Start, Stop, SetSpeed, Stopped, Recursive> Largest_t;

    typedef etl::packet<etl::imessage, Largest_t::size, Largest_t::alignment> Packet_t;

    etl::queue<Packet_t, 2> messageQueue;

    int startCount;
    int stopCount;
    int setSpeedCount;
    int unknownCount;
    int stoppedCount;
    bool isLampOn;
    int speed;
  };

  using MotorControlStateBase = StateBase<MotorControl>;

  //***********************************
  // The idle state.
  //***********************************
  class Idle : public MotorControlStateBase
  {
  public:
    Idle() : StateBase(StateId::IDLE) { }

    //***********************************
    etl::fsm_state_id_t on_enter_state() override
    {
      get_fsm_context().TurnRunningLampOff();
      return get_state_id();
    }

    //***********************************
    etl::fsm_state_id_t on_event(etl::imessage_router&, const Start&) override
    {
      ++get_fsm_context().startCount;
      return StateId::RUNNING;
    }

    //***********************************
    etl::fsm_state_id_t on_event(etl::imessage_router&, const Recursive&) override
    {
      get_fsm_context().queue_recursive_message(Start());
      return StateId::IDLE;
    }

    //***********************************
    etl::fsm_state_id_t on_event(etl::imessage_router&, const etl::imessage&) override
    {
      ++get_fsm_context().unknownCount;
      return get_state_id();
    }
  };

  //***********************************
  // The running state.
  //***********************************
  class Running : public MotorControlStateBase
  {
  public:
    Running() : StateBase(StateId::RUNNING) { }

    //***********************************
    etl::fsm_state_id_t on_enter_state() override
    {
      get_fsm_context().TurnRunningLampOn();

      return get_state_id();
    }

    //***********************************
    etl::fsm_state_id_t on_event(etl::imessage_router&, const Stop& event) override
    {
      ++get_fsm_context().stopCount;

      if (event.isEmergencyStop)
      {
        return StateId::IDLE;
      }
      else
      {
        return StateId::WINDING_DOWN;
      }
    }

    //***********************************
    etl::fsm_state_id_t on_event(etl::imessage_router&, const SetSpeed& event) override
    {
      ++get_fsm_context().setSpeedCount;
      get_fsm_context().SetSpeedValue(event.speed);
      return get_state_id();
    }

    //***********************************
    etl::fsm_state_id_t on_event(etl::imessage_router&, const etl::imessage&) override
    {
      ++get_fsm_context().unknownCount;
      return get_state_id();
    }

  };

  //***********************************
  // The winding down state.
  //***********************************
  class WindingDown : public MotorControlStateBase
  {
  public:
    WindingDown() : StateBase(StateId::WINDING_DOWN) { }

    //***********************************
    etl::fsm_state_id_t on_event(etl::imessage_router&, const Stopped&) override
    {
      ++get_fsm_context().stoppedCount;
      return StateId::IDLE;
    }

    //***********************************
    etl::fsm_state_id_t on_event(etl::imessage_router&, const etl::imessage&) override
    {
      ++get_fsm_context().unknownCount;
      return get_state_id();
    }
  };

  //***********************************
  // The locked state.
  //***********************************
  class Locked : public MotorControlStateBase
  {
  public:
    Locked() : StateBase(StateId::LOCKED) { }

    //***********************************
    etl::fsm_state_id_t on_event(etl::imessage_router&, const etl::imessage&) override
    {
      ++get_fsm_context().unknownCount;
      return get_state_id();
    }
  };

  // The states.
  Idle        idle;
  Running     running;
  WindingDown windingDown;
  Locked      locked;

  MotorControlStateBase* stateList[StateId::NUMBER_OF_STATES] =
  {
    &idle, &running, &windingDown, &locked
  };

  SUITE(test_map)
  {
    //*************************************************************************
    TEST(test_fsm)
    {
      etl::null_message_router nmr;

        // motorControl.reset();
        // motorControl.ClearStatistics();

      //CHECK(!motorControl.is_started());

      // Start the FSM.
      MotorControl motorControl(stateList, etl::size(stateList));
      motorControl.start(StateId::IDLE);
      CHECK(motorControl.is_started());

      // Now in Idle state.

      CHECK_EQUAL(StateId::IDLE, int(motorControl.get_state_id()));
      CHECK_EQUAL(StateId::IDLE, int(motorControl.get_state().get_state_id()));

      CHECK_EQUAL(false, motorControl.isLampOn);
      CHECK_EQUAL(0, motorControl.setSpeedCount);
      CHECK_EQUAL(0, motorControl.speed);
      CHECK_EQUAL(0, motorControl.startCount);
      CHECK_EQUAL(0, motorControl.stopCount);
      CHECK_EQUAL(0, motorControl.stoppedCount);
      CHECK_EQUAL(0, motorControl.unknownCount);

      // Send unhandled events.
      motorControl.receive(nmr, Stop());
      motorControl.receive(nmr, Stopped());
      motorControl.receive(nmr, SetSpeed(10));

      CHECK_EQUAL(StateId::IDLE, motorControl.get_state_id());
      CHECK_EQUAL(StateId::IDLE, motorControl.get_state().get_state_id());

      CHECK_EQUAL(false, motorControl.isLampOn);
      CHECK_EQUAL(0, motorControl.setSpeedCount);
      CHECK_EQUAL(0, motorControl.speed);
      CHECK_EQUAL(0, motorControl.startCount);
      CHECK_EQUAL(0, motorControl.stopCount);
      CHECK_EQUAL(0, motorControl.stoppedCount);
      CHECK_EQUAL(3, motorControl.unknownCount);

      // Send Start event.
      motorControl.receive(nmr, Start());

      // Now in Running state.

      CHECK_EQUAL(StateId::RUNNING, int(motorControl.get_state_id()));
      CHECK_EQUAL(StateId::RUNNING, int(motorControl.get_state().get_state_id()));

      CHECK_EQUAL(true, motorControl.isLampOn);
      CHECK_EQUAL(0, motorControl.setSpeedCount);
      CHECK_EQUAL(0, motorControl.speed);
      CHECK_EQUAL(1, motorControl.startCount);
      CHECK_EQUAL(0, motorControl.stopCount);
      CHECK_EQUAL(0, motorControl.stoppedCount);
      CHECK_EQUAL(3, motorControl.unknownCount);

      // Send unhandled events.
      motorControl.receive(nmr, Start());
      motorControl.receive(nmr, Stopped());

      CHECK_EQUAL(StateId::RUNNING, int(motorControl.get_state_id()));
      CHECK_EQUAL(StateId::RUNNING, int(motorControl.get_state().get_state_id()));

      CHECK_EQUAL(true, motorControl.isLampOn);
      CHECK_EQUAL(0, motorControl.setSpeedCount);
      CHECK_EQUAL(0, motorControl.speed);
      CHECK_EQUAL(1, motorControl.startCount);
      CHECK_EQUAL(0, motorControl.stopCount);
      CHECK_EQUAL(0, motorControl.stoppedCount);
      CHECK_EQUAL(5, motorControl.unknownCount);

      // Send SetSpeed event.
      motorControl.receive(nmr, SetSpeed(100));

      // Still in Running state.

      CHECK_EQUAL(StateId::RUNNING, int(motorControl.get_state_id()));
      CHECK_EQUAL(StateId::RUNNING, int(motorControl.get_state().get_state_id()));

      CHECK_EQUAL(true, motorControl.isLampOn);
      CHECK_EQUAL(1, motorControl.setSpeedCount);
      CHECK_EQUAL(100, motorControl.speed);
      CHECK_EQUAL(1, motorControl.startCount);
      CHECK_EQUAL(0, motorControl.stopCount);
      CHECK_EQUAL(0, motorControl.stoppedCount);
      CHECK_EQUAL(5, motorControl.unknownCount);

      // Send Stop event.
      motorControl.receive(nmr, Stop());

      // Now in WindingDown state.

      CHECK_EQUAL(StateId::WINDING_DOWN, int(motorControl.get_state_id()));
      CHECK_EQUAL(StateId::WINDING_DOWN, int(motorControl.get_state().get_state_id()));

      CHECK_EQUAL(true, motorControl.isLampOn);
      CHECK_EQUAL(1, motorControl.setSpeedCount);
      CHECK_EQUAL(100, motorControl.speed);
      CHECK_EQUAL(1, motorControl.startCount);
      CHECK_EQUAL(1, motorControl.stopCount);
      CHECK_EQUAL(0, motorControl.stoppedCount);
      CHECK_EQUAL(5, motorControl.unknownCount);

      // Send unhandled events.
      motorControl.receive(nmr, Start());
      motorControl.receive(nmr, Stop());
      motorControl.receive(nmr, SetSpeed(100));

      CHECK_EQUAL(StateId::WINDING_DOWN, int(motorControl.get_state_id()));
      CHECK_EQUAL(StateId::WINDING_DOWN, int(motorControl.get_state().get_state_id()));

      CHECK_EQUAL(true, motorControl.isLampOn);
      CHECK_EQUAL(1, motorControl.setSpeedCount);
      CHECK_EQUAL(100, motorControl.speed);
      CHECK_EQUAL(1, motorControl.startCount);
      CHECK_EQUAL(1, motorControl.stopCount);
      CHECK_EQUAL(0, motorControl.stoppedCount);
      CHECK_EQUAL(8, motorControl.unknownCount);

      // Send Stopped event.
      motorControl.receive(nmr, Stopped());

      // Now in Locked state via Idle state.
      CHECK_EQUAL(StateId::LOCKED, int(motorControl.get_state_id()));
      CHECK_EQUAL(StateId::LOCKED, int(motorControl.get_state().get_state_id()));

      CHECK_EQUAL(false, motorControl.isLampOn);
      CHECK_EQUAL(1, motorControl.setSpeedCount);
      CHECK_EQUAL(100, motorControl.speed);
      CHECK_EQUAL(1, motorControl.startCount);
      CHECK_EQUAL(1, motorControl.stopCount);
      CHECK_EQUAL(1, motorControl.stoppedCount);
      CHECK_EQUAL(8, motorControl.unknownCount);
    }

    //*************************************************************************
    TEST(test_fsm_emergency_stop)
    {
      etl::null_message_router nmr;

      // motorControl.reset();
      // motorControl.ClearStatistics();

      // CHECK(!motorControl.is_started());

      // Start the FSM.
      MotorControl motorControl(stateList, etl::size(stateList));
      motorControl.start(StateId::IDLE);
      CHECK(motorControl.is_started());

      // Now in Idle state.

      // Send Start event.
      motorControl.receive(nmr, Start());

      // Now in Running state.

      CHECK_EQUAL(StateId::RUNNING, int(motorControl.get_state_id()));
      CHECK_EQUAL(StateId::RUNNING, int(motorControl.get_state().get_state_id()));

      CHECK_EQUAL(true, motorControl.isLampOn);
      CHECK_EQUAL(0, motorControl.setSpeedCount);
      CHECK_EQUAL(0, motorControl.speed);
      CHECK_EQUAL(1, motorControl.startCount);
      CHECK_EQUAL(0, motorControl.stopCount);
      CHECK_EQUAL(0, motorControl.stoppedCount);
      CHECK_EQUAL(0, motorControl.unknownCount);

      // Send emergency Stop event.
      motorControl.receive(nmr, Stop(true));

      // Now in Locked state via Idle state.
      CHECK_EQUAL(StateId::LOCKED, int(motorControl.get_state_id()));
      CHECK_EQUAL(StateId::LOCKED, int(motorControl.get_state().get_state_id()));

      CHECK_EQUAL(false, motorControl.isLampOn);
      CHECK_EQUAL(0, motorControl.setSpeedCount);
      CHECK_EQUAL(0, motorControl.speed);
      CHECK_EQUAL(1, motorControl.startCount);
      CHECK_EQUAL(1, motorControl.stopCount);
      CHECK_EQUAL(0, motorControl.stoppedCount);
      CHECK_EQUAL(0, motorControl.unknownCount);
    }

    //*************************************************************************
    TEST(test_fsm_recursive_event)
    {
      etl::null_message_router nmr;

      // motorControl.reset();
      // motorControl.ClearStatistics();

      // motorControl.messageQueue.clear();

      // Start the FSM.
      MotorControl motorControl(stateList, etl::size(stateList));
      motorControl.start(StateId::IDLE);

      // Now in Idle state.
      // Send Start event.
      motorControl.receive(nmr, Recursive());

      CHECK_EQUAL(1U, motorControl.messageQueue.size());

      // Send the queued message.
      motorControl.receive(nmr, motorControl.messageQueue.front().get());
      motorControl.messageQueue.pop();

      // Now in Running state.

      CHECK_EQUAL(StateId::RUNNING, int(motorControl.get_state_id()));
      CHECK_EQUAL(StateId::RUNNING, int(motorControl.get_state().get_state_id()));

      CHECK_EQUAL(true, motorControl.isLampOn);
      CHECK_EQUAL(0, motorControl.setSpeedCount);
      CHECK_EQUAL(0, motorControl.speed);
      CHECK_EQUAL(1, motorControl.startCount);
      CHECK_EQUAL(0, motorControl.stopCount);
      CHECK_EQUAL(0, motorControl.stoppedCount);
      CHECK_EQUAL(0, motorControl.unknownCount);
    }

    //*************************************************************************
    TEST(test_fsm_supported)
    {
      MotorControl motorControl(stateList, etl::size(stateList));
      motorControl.start(StateId::IDLE);
      CHECK(motorControl.accepts(EventId::SET_SPEED));
      CHECK(motorControl.accepts(EventId::START));
      CHECK(motorControl.accepts(EventId::STOP));
      CHECK(motorControl.accepts(EventId::STOPPED));
      CHECK(motorControl.accepts(EventId::UNSUPPORTED));

      CHECK(motorControl.accepts(SetSpeed(0)));
      CHECK(motorControl.accepts(Start()));
      CHECK(motorControl.accepts(Stop()));
      CHECK(motorControl.accepts(Stopped()));
      CHECK(motorControl.accepts(Unsupported()));
    }
  };
}
