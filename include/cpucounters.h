/*
Copyright (c) 2009-2011, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// written by Roman Dementiev
//

#ifndef CPUCOUNTERS_HEADER
#define CPUCOUNTERS_HEADER

/*!     \file cpucounters.h
        \brief Main CPU counters header

        Include this header file if you want to access CPU counters (core and uncore - including memory controller chips)
*/

#ifndef INTELPCM_API
#define INTELPCM_API
#endif

#include "types.h"
#include "msr.h"
#include <vector>

#ifndef _MSC_VER
#include <semaphore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

#ifdef _DARWIN_VER
#define NUM_INSTANCE_SEMAPHORE_NAME "/IntelPCMInstances"
#define GLOBAL_SEMAPHORE_NAME "/IntelPCMonitorSystemLock"
#else
#define GLOBAL_SEMAPHORE_NAME "/Intel(r) Performance Counter Monitor instance create-destroy lock"
#define NUM_INSTANCE_SEMAPHORE_NAME "/Number of running Intel(r) Performance Counter Monitor instances"
#endif

class SystemCounterState;
class SocketCounterState;
class CoreCounterState;

/*
        CPU performance monitoring routines

        A set of performance monitoring routines for recent Intel CPUs
*/

struct INTELPCM_API TopologyEntry // decribes a core
{
    int32 os_id;
    int32 socket;
    int32 core_id;

    TopologyEntry() : os_id(-1), socket(-1), core_id(-1) { }
};

//template class INTELPCM_API std::allocator<TopologyEntry>;
//template class INTELPCM_API std::vector<TopologyEntry>;

/*!
        \brief CPU Performance Monitor

        This singleton object needs to be instantiated for each process
        before accessing counting and measuring routines
*/
class INTELPCM_API PCM
{
    PCM();     // forbidden to call directly because it is a singleton

    int32 cpu_family;
    int32 cpu_model;
    int32 threads_per_core;
    int32 num_cores;
    int32 num_sockets;
    uint32 core_gen_counter_num_max;
    uint32 core_gen_counter_num_used;
    uint32 core_gen_counter_width;
    uint32 core_fixed_counter_num_max;
    uint32 core_fixed_counter_num_used;
    uint32 core_fixed_counter_width;
    uint32 uncore_gen_counter_num_max;
    uint32 uncore_gen_counter_num_used;
    uint32 uncore_gen_counter_width;
    uint32 uncore_fixed_counter_num_max;
    uint32 uncore_fixed_counter_num_used;
    uint32 uncore_fixed_counter_width;
    int32 perfmon_version;
    int32 perfmon_config_anythread;
    uint64 nominal_frequency;
    uint64 qpi_speed; // in GBytes/second

    std::vector<TopologyEntry> topology;
    std::string errorMessage;

    static PCM * instance;
    MsrHandle ** MSR;

public:
    //! Mode of programming (parameter in the program() method)
    enum ProgramMode {
        DEFAULT_EVENTS = 0,         /*!< Default choice of events, the additional parameter is not needed and ignored */
        CUSTOM_CORE_EVENTS = 1,     /*!< Custom set of core events specified in the parameter to the program method. The parameter must be a pointer to array of four \c CustomCoreEventDescription values */
        EXT_CUSTOM_CORE_EVENTS = 2, /*!< Custom set of core events specified in the parameter to the program method. The parameter must be a pointer to a \c ExtendedCustomCoreEventDescription  data structure */
        INVALID_MODE                /*!< Non-programmed mode */
    };

    //! Return codes (e.g. for program(..) method)
    enum ErrorCode {
        Success = 0,
        MSRAccessDenied = 1,
        PMUBusy = 2,
        UnknownError
    };

    /*! \brief Custom Core event description

        See "Intel 64 and IA-32 Architectures Software Developers Manual Volume 3B:
        System Programming Guide, Part 2" for the concrete values of the data structure fields,
        e.g. Appendix A.2 "Performance Monitoring Events for Intel(r) Core(tm) Processor Family
        and Xeon Processor Family"
    */
    struct CustomCoreEventDescription
    {
        int32 event_number, umask_value;
    };
    
    /*! \brief Extended custom core event description

        In contrast to CustomCoreEventDescription supports configuration of all fields.
        
        See "Intel 64 and IA-32 Architectures Software Developers Manual Volume 3B:
        System Programming Guide, Part 2" for the concrete values of the data structure fields,
        e.g. Appendix A.2 "Performance Monitoring Events for Intel(r) Core(tm) Processor Family
        and Xeon Processor Family"
    */
    struct ExtendedCustomCoreEventDescription
    {
        FixedEventControlRegister * fixedCfg; // if NULL, then default configuration performed for fixed counters
	uint32 nGPCounters; // number of general purpose counters
	EventSelectRegister * gpCounterCfg; // general purpose counters, if NULL, then default configuration performed for GP counters
    };

private:
    ProgramMode mode;
    CustomCoreEventDescription coreEventDesc[4];

        #ifdef _MSC_VER
    HANDLE numInstancesSemaphore;     // global semaphore that counts the number of PCM instances on the system
        #else
    // global semaphore that counts the number of PCM instances on the system
    sem_t * numInstancesSemaphore;
        #endif

    bool PMUinUse();
    void cleanupPMU();
    bool decrementInstanceSemaphore(); // returns true if it was the last instance
    void computeQPIspeed(int core_nr);

public:
    /*!
            \brief Returns PCM object

            Returns PCM object. If the PCM has not been created before than
            an instance is created. PCM is a singleton.

            \return Pointer to PCM object
    */
    static PCM * getInstance();        // the only way to get access

    /*!
            \brief Checks the status of PCM object

            Call this method to check if PCM gained access to model specific registers. The method is deprecated, see program error code instead.

            \return true iff access to model specific registers works without problems
    */
    bool good();                       // true if access to CPU counters works

    /*! \brief Returns the error message

                Call this when good() returns false, otherwise return an empty string
    */
    const std::string & getErrorMessage() const
    {
        return errorMessage;
    }

    /*! \brief Programs performance counters
        \param mode_ mode of programming, see ProgramMode definition
        \param parameter_ optional parameter for some of programming modes

                Call this method before you start using the performance counting routines.

        \warning Using this routines with other tools that *program* Performance Monitoring
        Units (PMUs) on CPUs is not recommended because PMU can not be shared. Tools that are known to
        program PMUs: Intel(r) VTune(tm), Intel(r) Performance Tuning Utility (PTU). This code may make
        VTune or PTU measurements invalid. VTune or PTU measurement may make measurement with this code invalid. Please enable either usage of these routines or VTune/PTU/etc.
    */
    ErrorCode program(ProgramMode mode_ = DEFAULT_EVENTS, void * parameter_ = NULL); // program counters and start counting

    /*! \brief Cleanups resources and stops performance counting

            One needs to call this method when your program finishes or/and you are not going to use the
            performance counting routines anymore.
*/
    void cleanup();

/*! \brief Forces PMU reset

            If there is no chance to free up PMU from other applications you might try to call this method at your own risk.
*/
    void resetPMU();

    /*! \brief Reads the counter state of the system

            System consists of several sockets (CPUs).
            Socket has a CPU in it. Socket (CPU) consists of several (logical) cores.

            \return State of counters in the entire system
    */
    SystemCounterState getSystemCounterState();

    /*! \brief Reads the counter state of a socket
            \param socket socket id
            \return State of counters in the socket
    */
    SocketCounterState getSocketCounterState(uint32 socket);

    /*! \brief Reads the counter state of a (logical) core

        Be aware that during the measurement other threads may be scheduled on the same core by the operating system (this is called context-switching). The performance events caused by these threads will be counted as well.


            \param core core id
            \return State of counters in the core
    */
    CoreCounterState getCoreCounterState(uint32 core);

    /*! \brief Reads number of logical cores in the system
            \return Number of logical cores in the system
    */
    uint32 getNumCores();

    /*! \brief Reads number of sockets (CPUs) in the system
            \return Number of sockets in the system
    */
    uint32 getNumSockets();

    /*! \brief Reads how many hardware threads has a physical core
            "Hardware thread" is a logical core in a different terminology.
            If Intel(r) Hyperthreading(tm) is enabled then this function returns 2.
            \return Number of hardware threads per physical core
    */
    uint32 getThreadsPerCore();

	// [Dec 1, 2011 7:13:11 PM] [amr.eladawy] [extra getters for PCM info. used in the JNI DLL]
	int32 getCpuFamily();
	int32 getCpuModel();
	uint32 getCoreGenCounterNumMax();
	uint32 getCoreGenCounterNumUsed();
	uint32 getCoreGenCounterWidth();
	uint32 getCoreFixedCounterNumMax();
	uint32 getCoreFixedCounterNumUsed();
	uint32 getCoreFixedCounterWidth();
	uint32 getUncoreGenCounterNumMax();
	uint32 getUncoreGenCounterNumUsed();
	uint32 getUncoreGenCounterWidth();
	uint32 getUncoreFixedCounterNumMax();
	uint32 getUncoreFixedCounterNumUsed();
	uint32 getUncoreFixedCounterWidth();
	uint32 getPerfmonConfigAnythread();
	uint32 getPerfmonVersion();
	
	uint64 getQpiSpeed();
		

	/*! \brief Checks if SMT (HyperThreading) is enabled.
	\return true iff SMT (HyperThreading) is enabled.
	*/
	bool getSMT();                // returns true iff SMT ("Hyperthreading") is on

    /*! \brief Reads the nominal core frequency
            \return Nominal frequency in Hz
    */
    uint64 getNominalFrequency(); // in Hz

    //! \brief Identifiers of supported CPU models
    enum SupportedCPUModels
    {
        NEHALEM_EP = 26,
        NEHALEM_EP_2 = 30,
        ATOM = 28,
        CLARKDALE = 37,
        WESTMERE_EP = 44,
        NEHALEM_EX = 46,
        WESTMERE_EX = 47,
        SANDY_BRIDGE = 42
    };

    //! \brief Reads CPU model id
    //! \return CPU model ID
    uint32 getCPUModel() { return cpu_model; }

    //! \brief Determines socket of given core
    //! \param core_id core identifier
    //! \return socket identifier
    uint32 getSocketId(uint32 core_id)
    {
        return topology[core_id].socket;
    }

    //! \brief Returns the number of Intel(r) Quick Path Interconnect(tm) links per socket
    //! \return number of QPI links per socket
    uint64 getQPILinksPerSocket() const
    {
        switch (cpu_model)
        {
        case NEHALEM_EP:
        case WESTMERE_EP:
        case CLARKDALE:
            if (num_sockets == 2)
                return 2;
            else
                return 1;
        case NEHALEM_EX:
        case WESTMERE_EX:
            return 4;
        }
        return 0;
    }

    //! \brief Returns the max number of instructions per cycle
    //! \return max number of instructions per cycle
    uint32 getMaxIPC() const
    {
        switch (cpu_model)
        {
        case NEHALEM_EP:
        case WESTMERE_EP:
        case NEHALEM_EX:
        case WESTMERE_EX:
        case CLARKDALE:
        case SANDY_BRIDGE:
            return 4;
        case ATOM:
            return 2;
        }
        return 0;
    }

    //! \brief Return TSC timer value in time units
    //! \param multiplier use 1 for seconds, 1000 for ms, 1000000 for mks, etc (default is 1000: ms)
    //! \param core core to read on-chip TSC value (default is 0)
    //! \return time counter value
    uint64 getTickCount(uint64 multiplier = 1000 /* ms */, uint32 core = 0);

    //! \brief Return TSC timer value in time units using rdtscp instruction from current core
    //! \param multiplier use 1 for seconds, 1000 for ms, 1000000 for mks, etc (default is 1000: ms)
	//! \warning Processor support is required  bit 27 of cpuid EDX must be set, for Windows, Visual Studio 2010 is required
    //! \return time counter value
    uint64 getTickCountRDTSCP(uint64 multiplier = 1000 /* ms */);


    //! \brief Return QPI Link Speed in GBytes/second
    //! \warning Works only for Nehalem-EX (Xeon 7500) and Westmere-EX (Xeon E7) processors
    //! \return QPI Link Speed in GBytes/second
    uint64 getQPILinkSpeed() { return qpi_speed; }

	static bool initWinRing0Lib();

    ~PCM();
};

//! \brief Basic core counter state
//!
//! Intended only for derivation, but not for the direct use
class BasicCounterState
{
    friend class PCM;
    template <class CounterStateType>
    friend double getExecUsage(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend double getIPC(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend double getAverageFrequency(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend double getActiveAverageFrequency(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend double getCyclesLostDueL3CacheMisses(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend double getCyclesLostDueL2CacheMisses(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend double getRelativeFrequency(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend double getActiveRelativeFrequency(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend double getL2CacheHitRatio(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend double getL3CacheHitRatio(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend uint64 getL3CacheMisses(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend uint64 getL2CacheMisses(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend uint64 getL2CacheHits(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend uint64 getCycles(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend uint64 getInstructionsRetired(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend uint64 getCycles(const CounterStateType & now);
    template <class CounterStateType>
    friend uint64 getInstructionsRetired(const CounterStateType & now);
    template <class CounterStateType>
    friend uint64 getL3CacheHitsNoSnoop(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend uint64 getL3CacheHitsSnoop(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend uint64 getL3CacheHits(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend uint64 getNumberOfCustomEvents(int32 eventCounterNr, const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend uint64 getInvariantTSC(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend uint64 getRefCycles(const CounterStateType & before, const CounterStateType & after);

public:
    uint64 InstRetiredAny;
    uint64 CpuClkUnhaltedThread;
    uint64 CpuClkUnhaltedRef;
    union {
        uint64 L3Miss;
        uint64 Event0;
        uint64 ArchLLCRef;
    };
    union {
        uint64 L3UnsharedHit;
        uint64 Event1;
        uint64 ArchLLCMiss;
    };
    union {
        uint64 L2HitM;
        uint64 Event2;
    };
    union {
        uint64 L2Hit;
        uint64 Event3;
    };
    uint64 InvariantTSC; // invariant time stamp counter

    void readAndAggregate(MsrHandle *);

public:
    BasicCounterState() : InstRetiredAny(0), CpuClkUnhaltedThread(0), CpuClkUnhaltedRef(0), L3Miss(0), L3UnsharedHit(0), L2HitM(0), L2Hit(0), InvariantTSC(0) { }
    virtual ~BasicCounterState() { }
};

//! \brief Basic uncore counter state
//!
//! Intended only for derivation, but not for the direct use
class UncoreCounterState
{
    friend class PCM;
    template <class CounterStateType>
    friend uint64 getBytesReadFromMC(const CounterStateType & before, const CounterStateType & after);
    template <class CounterStateType>
    friend uint64 getBytesWrittenToMC(const CounterStateType & before, const CounterStateType & after);

public:
    uint64 UncMCFullWrites;
    uint64 UncMCNormalReads;

    void readAndAggregate(MsrHandle *);

public:
    UncoreCounterState() : UncMCFullWrites(0), UncMCNormalReads(0) { }
    virtual ~UncoreCounterState() { }
};

//! \brief (Logical) core-wide counter state
class CoreCounterState : public BasicCounterState
{
    friend class PCM;

public:
};

//! \brief Socket-wide counter state
class SocketCounterState : public BasicCounterState, public UncoreCounterState
{
    friend class PCM;

protected:
    void readAndAggregate(MsrHandle * handle)
    {
        BasicCounterState::readAndAggregate(handle);
        UncoreCounterState::readAndAggregate(handle);
    }

public:

};

//! \brief System-wide counter state
class SystemCounterState : public BasicCounterState, public UncoreCounterState
{
    friend class PCM;
public:
    std::vector<std::vector<uint64> > incomingQPIPackets;
    std::vector<std::vector<uint64> > outgoingQPIIdleFlits;
public:
    uint64 uncoreTSC;

protected:
    void readAndAggregate(MsrHandle * handle)
    {
        BasicCounterState::readAndAggregate(handle);
        UncoreCounterState::readAndAggregate(handle);
    }

public:
    friend uint64 getIncomingQPILinkBytes(uint32 socketNr, uint32 linkNr, const SystemCounterState & before, const SystemCounterState & after);
    friend uint64 getIncomingQPILinkBytes(uint32 socketNr, uint32 linkNr, const SystemCounterState & now);
    friend double getOutgoingQPILinkUtilization(uint32 socketNr, uint32 linkNr, const SystemCounterState & before, const SystemCounterState & after);
    friend uint64 getOutgoingQPILinkBytes(uint32 socketNr, uint32 linkNr, const SystemCounterState & before, const SystemCounterState & after);
    friend uint64 getOutgoingQPILinkBytes(uint32 socketNr, uint32 linkNr, const SystemCounterState & now);

    SystemCounterState() :
        uncoreTSC(0)
    {
        PCM * m = PCM::getInstance();
        incomingQPIPackets.resize(m->getNumSockets(),
                                  std::vector<uint64>((uint32)m->getQPILinksPerSocket(), 0));
        outgoingQPIIdleFlits.resize(m->getNumSockets(),
                                    std::vector<uint64>((uint32)m->getQPILinksPerSocket(), 0));
    }
};

/*! \brief Reads the counter state of the system

        Helper function. Uses PCM object to access counters.

        System consists of several sockets (CPUs).
        Socket has a CPU in it. Socket (CPU) consists of several (logical) cores.

        \return State of counters in the entire system
*/
INTELPCM_API SystemCounterState getSystemCounterState();

/*! \brief Reads the counter state of a socket

        Helper function. Uses PCM object to access counters.

        \param socket socket id
        \return State of counters in the socket
*/
INTELPCM_API SocketCounterState getSocketCounterState(uint32 socket);

/*! \brief Reads the counter state of a (logical) core

    Helper function. Uses PCM object to access counters.

    \param core core id
    \return State of counters in the core
*/
INTELPCM_API CoreCounterState getCoreCounterState(uint32 core);


/*! \brief Computes average number of retired instructions per core cycle (IPC)

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return IPC
*/
template <class CounterStateType>
double getIPC(const CounterStateType & before, const CounterStateType & after) // instructions per cycle
{
    int64 clocks = after.CpuClkUnhaltedThread - before.CpuClkUnhaltedThread;
    if (clocks != 0)
        return double(after.InstRetiredAny - before.InstRetiredAny) / double(clocks);
    return -1;
}


/*! \brief Computes the number of retired instructions

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return number of retired instructions
*/
template <class CounterStateType>
uint64 getInstructionsRetired(const CounterStateType & before, const CounterStateType & after) // instructions
{
    return after.InstRetiredAny - before.InstRetiredAny;
}

/*! \brief Computes average number of retired instructions per time intervall

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return usage
*/
template <class CounterStateType>
double getExecUsage(const CounterStateType & before, const CounterStateType & after) // usage
{
    int64 timer_clocks = after.InvariantTSC - before.InvariantTSC;
    if (timer_clocks != 0)
        return double(after.InstRetiredAny - before.InstRetiredAny) / double(timer_clocks);
    return -1;
}

/*! \brief Computes the number of retired instructions

    \param now Current CPU counter state
    \return number of retired instructions
*/
template <class CounterStateType>
uint64 getInstructionsRetired(const CounterStateType & now) // instructions
{
    return now.InstRetiredAny;
}

/*! \brief Computes the number core clock cycles when signal on a specific core is running (not halted)

    Returns number of used cycles (halted cyles are not counted).
    The counter does not advance in the following conditions:
    - an ACPI C-state is other than C0 for normal operation
    - HLT
    - STPCLK+ pin is asserted
    - being throttled by TM1
    - during the frequency switching phase of a performance state transition

    The performance counter for this event counts across performance state
    transitions using different core clock frequencies

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return number core clock cycles
*/
template <class CounterStateType>
uint64 getCycles(const CounterStateType & before, const CounterStateType & after) // clocks
{
    return after.CpuClkUnhaltedThread - before.CpuClkUnhaltedThread;
}

/*! \brief Computes the number of reference clock cycles while clock signal on the core is running

    The reference clock operates at a fixed frequency, irrespective of core
    frequency changes due to performance state transitions. See Intel(r) Software
    Developer's Manual for more details

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return number core clock cycles
*/
template <class CounterStateType>
uint64 getRefCycles(const CounterStateType & before, const CounterStateType & after) // clocks
{
    return after.CpuClkUnhaltedRef - before.CpuClkUnhaltedRef;
}

/*! \brief Computes the number executed core clock cycles

    Returns number of used cycles (halted cyles are not counted).

    \param now Current CPU counter state
    \return number core clock cycles
*/
template <class CounterStateType>
uint64 getCycles(const CounterStateType & now) // clocks
{
    return now.CpuClkUnhaltedThread;
}

/*! \brief Computes average number of retired instructions per core cycle for the entire system combining instruction counts from logical cores to corresponding physical cores

        Use this metric to evaluate IPC improvement between SMT(Hyperthreading) on and SMT off.

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return IPC
*/
inline double getCoreIPC(const SystemCounterState & before, const SystemCounterState & after) // instructions per cycle
{
    double ipc = getIPC(before, after);
    PCM * m = PCM::getInstance();
    if (ipc >= 0. && m)
        return ipc * double(m->getThreadsPerCore());
    return -1;
}


/*! \brief Computes average number of retired instructions per time intervall for the entire system combining instruction counts from logical cores to corresponding physical cores

        Use this metric to evaluate cores utilization improvement between SMT(Hyperthreading) on and SMT off.

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return usage
*/
inline double getTotalExecUsage(const SystemCounterState & before, const SystemCounterState & after) // usage
{
    double usage = getExecUsage(before, after);
    PCM * m = PCM::getInstance();
    if (usage >= 0. && m)
        return usage * double(m->getThreadsPerCore());
    return -1;
}

/*! \brief Computes average core frequency also taking Intel Turbo Boost technology into account

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return frequency in Hz
*/
template <class CounterStateType>
double getAverageFrequency(const CounterStateType & before, const CounterStateType & after) // in Hz
{
    int64 clocks = after.CpuClkUnhaltedThread - before.CpuClkUnhaltedThread;
    int64 timer_clocks = after.InvariantTSC - before.InvariantTSC;
    PCM * m = PCM::getInstance();
    if (timer_clocks != 0 && m)
        return double(m->getNominalFrequency()) * double(clocks) / double(timer_clocks);
    return -1;
}

/*! \brief Computes average core frequency when not in powersaving C0-state (also taking Intel Turbo Boost technology into account)

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return frequency in Hz
*/
template <class CounterStateType>
double getActiveAverageFrequency(const CounterStateType & before, const CounterStateType & after) // in Hz
{
    int64 clocks = after.CpuClkUnhaltedThread - before.CpuClkUnhaltedThread;
    int64 ref_clocks = after.CpuClkUnhaltedRef - before.CpuClkUnhaltedRef;
    PCM * m = PCM::getInstance();
    if (ref_clocks != 0 && m)
        return double(m->getNominalFrequency()) * double(clocks) / double(ref_clocks);
    return -1;
}

/*! \brief Computes average core frequency also taking Intel Turbo Boost technology into account

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return Fraction of nominal frequency
*/
template <class CounterStateType>
double getRelativeFrequency(const CounterStateType & before, const CounterStateType & after) // fraction of nominal frequency
{
    int64 clocks = after.CpuClkUnhaltedThread - before.CpuClkUnhaltedThread;
    int64 timer_clocks = after.InvariantTSC - before.InvariantTSC;
    if (timer_clocks != 0)
        return double(clocks) / double(timer_clocks);
    return -1;
}

/*! \brief Computes average core frequency when not in powersaving C0-state (also taking Intel Turbo Boost technology into account)

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return Fraction of nominal frequency (if >1.0 then Turbo was working during the measurement)
*/
template <class CounterStateType>
double getActiveRelativeFrequency(const CounterStateType & before, const CounterStateType & after) // fraction of nominal frequency
{
    int64 clocks = after.CpuClkUnhaltedThread - before.CpuClkUnhaltedThread;
    int64 ref_clocks = after.CpuClkUnhaltedRef - before.CpuClkUnhaltedRef;
    if (ref_clocks != 0)
        return double(clocks) / double(ref_clocks);
    return -1;
}

/*! \brief Estimates how many core cycles were potentially lost due to L3 cache misses.

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \warning Works only in the DEFAULT_EVENTS programming mode (see program() method)
    \return ratio that is usually beetween 0 and 1 ; in some cases could be >1.0 due to a lower memory latency estimation
*/
template <class CounterStateType>
double getCyclesLostDueL3CacheMisses(const CounterStateType & before, const CounterStateType & after) // 0.0 - 1.0
{
	if (PCM::getInstance()->getCPUModel() == PCM::ATOM) return -1;
    int64 clocks = after.CpuClkUnhaltedThread - before.CpuClkUnhaltedThread;
    if (clocks != 0)
    {
        return 180. * double(after.L3Miss - before.L3Miss) / double(clocks);
    }
    return -1;
}

/*! \brief Estimates how many core cycles were potentially lost due to missing L2 cache but still hitting L3 cache

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \warning Works only in the DEFAULT_EVENTS programming mode (see program() method)
        \warning Currently not supported on Intel(R) Atom(tm) processor
    \return ratio that is usually beetween 0 and 1 ; in some cases could be >1.0 due to a lower access latency estimation
*/
template <class CounterStateType>
double getCyclesLostDueL2CacheMisses(const CounterStateType & before, const CounterStateType & after) // 0.0 - 1.0
{
    if (PCM::getInstance()->getCPUModel() == PCM::ATOM) return -1;
    int64 clocks = after.CpuClkUnhaltedThread - before.CpuClkUnhaltedThread;
    if (clocks != 0)
    {
        double L3UnsharedHit = (double)(after.L3UnsharedHit - before.L3UnsharedHit);
        double L2HitM = (double)(after.L2HitM - before.L2HitM);
        return (35. * L3UnsharedHit + 74. * L2HitM) / double(clocks);
    }
    return -1;
}

/*! \brief Computes L2 cache hit ratio

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \warning Works only in the DEFAULT_EVENTS programming mode (see program() method)
    \return value between 0 and 1
*/
template <class CounterStateType>
double getL2CacheHitRatio(const CounterStateType & before, const CounterStateType & after) // 0.0 - 1.0
{
    if (PCM::getInstance()->getCPUModel() == PCM::ATOM)
    {
        uint64 L2Miss = after.ArchLLCMiss - before.ArchLLCMiss;
        uint64 L2Ref = after.ArchLLCRef - before.ArchLLCRef;
        if (L2Ref) return 1. - (double(L2Miss) / double(L2Ref));
        return 1;
    }
    uint64 L3Miss = after.L3Miss - before.L3Miss;
    uint64 L3UnsharedHit = after.L3UnsharedHit - before.L3UnsharedHit;
    uint64 L2HitM = after.L2HitM - before.L2HitM;
    uint64 L2Hit = after.L2Hit - before.L2Hit;
    uint64 hits = L2Hit;
    uint64 all = L2Hit + L2HitM + L3UnsharedHit + L3Miss;
    if (all) return double(hits) / double(all);

    return 1;
}

/*! \brief Computes L3 cache hit ratio

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \warning Works only in the DEFAULT_EVENTS programming mode (see program() method)
    \return value between 0 and 1
*/
template <class CounterStateType>
double getL3CacheHitRatio(const CounterStateType & before, const CounterStateType & after) // 0.0 - 1.0
{
    if (PCM::getInstance()->getCPUModel() == PCM::ATOM) return -1;

    uint64 L3Miss = after.L3Miss - before.L3Miss;
    uint64 L3UnsharedHit = after.L3UnsharedHit - before.L3UnsharedHit;
    uint64 L2HitM = after.L2HitM - before.L2HitM;
    uint64 hits = L3UnsharedHit + L2HitM;
    uint64 all = L2HitM + L3UnsharedHit + L3Miss;
    if (all) return double(hits) / double(all);

    return 1;
}

/*! \brief Computes number of L3 cache misses

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \warning Works only in the DEFAULT_EVENTS programming mode (see program() method)
    \return number of misses
*/
template <class CounterStateType>
uint64 getL3CacheMisses(const CounterStateType & before, const CounterStateType & after)
{
    if (PCM::getInstance()->getCPUModel() == PCM::ATOM) return 0;
    return after.L3Miss - before.L3Miss;
}

/*! \brief Computes number of L2 cache misses

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \warning Works only in the DEFAULT_EVENTS programming mode (see program() method)
    \return number of misses
*/
template <class CounterStateType>
uint64 getL2CacheMisses(const CounterStateType & before, const CounterStateType & after)
{
    if (PCM::getInstance()->getCPUModel() == PCM::ATOM)
    {
        return after.ArchLLCMiss - before.ArchLLCMiss;
    }
    uint64 L3Miss = after.L3Miss - before.L3Miss;
    uint64 L3UnsharedHit = after.L3UnsharedHit - before.L3UnsharedHit;
    uint64 L2HitM = after.L2HitM - before.L2HitM;
    return L2HitM + L3UnsharedHit + L3Miss;
}

/*! \brief Computes number of L2 cache hits

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \warning Works only in the DEFAULT_EVENTS programming mode (see program() method)
    \return number of hits
*/
template <class CounterStateType>
uint64 getL2CacheHits(const CounterStateType & before, const CounterStateType & after)
{
    if (PCM::getInstance()->getCPUModel() == PCM::ATOM)
    {
        uint64 L2Miss = after.ArchLLCMiss - before.ArchLLCMiss;
        uint64 L2Ref = after.ArchLLCRef - before.ArchLLCRef;
        return L2Ref - L2Miss;
    }
    return after.L2Hit - before.L2Hit;
}

/*! \brief Computes number of L3 cache hits where no snooping in sibling L2 caches had to be done

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \warning Works only in the DEFAULT_EVENTS programming mode (see program() method)
    \return number of hits
*/
template <class CounterStateType>
uint64 getL3CacheHitsNoSnoop(const CounterStateType & before, const CounterStateType & after)
{
    if (PCM::getInstance()->getCPUModel() == PCM::ATOM) return 0;
    return after.L3UnsharedHit - before.L3UnsharedHit;
}

/*! \brief Computes number of L3 cache hits where snooping in sibling L2 caches had to be done

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \warning Works only in the DEFAULT_EVENTS programming mode (see program() method)
    \return number of hits
*/
template <class CounterStateType>
uint64 getL3CacheHitsSnoop(const CounterStateType & before, const CounterStateType & after)
{
    if (PCM::getInstance()->getCPUModel() == PCM::ATOM) return 0;
    return after.L2HitM - before.L2HitM;
}


/*! \brief Computes total number of L3 cache hits

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \warning Works only in the DEFAULT_EVENTS programming mode (see program() method)
    \return number of hits
*/
template <class CounterStateType>
uint64 getL3CacheHits(const CounterStateType & before, const CounterStateType & after)
{
    if (PCM::getInstance()->getCPUModel() == PCM::ATOM) return 0;
    return getL3CacheHitsSnoop(before, after) + getL3CacheHitsNoSnoop(before, after);
}

/*! \brief Computes number of invariant time stamp counter ticks

    This counter counts irrespectively of C-, P- or T-states

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return number of time stamp counter ticks
*/
template <class CounterStateType>
uint64 getInvariantTSC(const CounterStateType & before, const CounterStateType & after)
{
    return after.InvariantTSC - before.InvariantTSC;
}


/*! \brief Computes residency in the C0 (active,non-halted) core state

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return residence ratio (0..1): 0 - 0%, 1.0 - 100%
*/
template <class CounterStateType>
double getC0Residency(const CounterStateType & before, const CounterStateType & after)
{
   return double(getRefCycles(before,after))/double(getInvariantTSC(before,after));
}


/*! \brief Computes number of bytes read from DRAM memory controllers

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return Number of bytes
*/
template <class CounterStateType>
uint64 getBytesReadFromMC(const CounterStateType & before, const CounterStateType & after)
{
    return (after.UncMCNormalReads - before.UncMCNormalReads) * 64;
}

/*! \brief Computes number of bytes written to DRAM memory controllers

    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return Number of bytes
*/
template <class CounterStateType>
uint64 getBytesWrittenToMC(const CounterStateType & before, const CounterStateType & after)
{
    return (after.UncMCFullWrites - before.UncMCFullWrites) * 64;
}

/*! \brief Returns the number of occured custom core events

    Read number of events programmed with the \c CUSTOM_CORE_EVENTS

    \param eventCounterNr Event/counter number (value from 0 to 3)
    \param before CPU counter state before the experiment
    \param after CPU counter state after the experiment
    \return Number of bytes
*/
template <class CounterStateType>
uint64 getNumberOfCustomEvents(int32 eventCounterNr, const CounterStateType & before, const CounterStateType & after)
{
    return ((&after.Event0)[eventCounterNr] - (&before.Event0)[eventCounterNr]);
}

/*! \brief Get estimation of QPI data traffic per incoming QPI link

    Returns an estimation of number of data bytes transferred to a socket over Intel(r) Quick Path Interconnect

    \param socketNr socket identifier
    \param linkNr linkNr
    \param before System CPU counter state before the experiment
    \param after System CPU counter state after the experiment
    \return Number of bytes
*/
inline uint64 getIncomingQPILinkBytes(uint32 socketNr, uint32 linkNr, const SystemCounterState & before, const SystemCounterState & after)
{
    uint64 b = before.incomingQPIPackets[socketNr][linkNr];
    uint64 a = after.incomingQPIPackets[socketNr][linkNr];
    // prevent overflows due to counter dissynchronisation
    return (a > b) ? (64 * (a - b)) : 0;
}

/*! \brief Get data utilization of incoming QPI link (0..1)

    Returns an estimation of utilization of QPI link by data traffic transferred to a socket over Intel(r) Quick Path Interconnect

    \param socketNr socket identifier
    \param linkNr linkNr
    \param before System CPU counter state before the experiment
    \param after System CPU counter state after the experiment
    \return utilization (0..1)
*/
inline double getIncomingQPILinkUtilization(uint32 socketNr, uint32 linkNr, const SystemCounterState & before, const SystemCounterState & after)
{
    PCM * m = PCM::getInstance();
    const int cpu_model = m->getCPUModel();
    if (cpu_model != PCM::NEHALEM_EX && cpu_model != PCM::WESTMERE_EX) return 0.;

    const double bytes = (double)getIncomingQPILinkBytes(socketNr, linkNr, before, after);
    const uint64 max_speed = m->getQPILinkSpeed();
    const double max_bytes = (double)(double(max_speed) * double(getInvariantTSC(before, after) / double(m->getNumCores())) / double(m->getNominalFrequency()));
    return bytes / max_bytes;
}

/*! \brief Get utilization of outgoing QPI link (0..1)

    Returns an estimation of utilization of QPI link by (data+nondata) traffic transferred from a socket over Intel(r) Quick Path Interconnect

    \param socketNr socket identifier
    \param linkNr linkNr
    \param before System CPU counter state before the experiment
    \param after System CPU counter state after the experiment
    \return utilization (0..1)
*/
inline double getOutgoingQPILinkUtilization(uint32 socketNr, uint32 linkNr, const SystemCounterState & before, const SystemCounterState & after)
{
    PCM * m = PCM::getInstance();
    const int cpu_model = m->getCPUModel();
    if (cpu_model != PCM::NEHALEM_EX && cpu_model != PCM::WESTMERE_EX) return 0.;
    const uint64 b = before.outgoingQPIIdleFlits[socketNr][linkNr];
    const uint64 a = after.outgoingQPIIdleFlits[socketNr][linkNr];
    // prevent overflows due to counter dissynchronisation
    const double idle_flits = (double)((a > b) ? (a - b) : 0);
    const uint64 bTSC = before.uncoreTSC;
    const uint64 aTSC = after.uncoreTSC;
    const double tsc = (double)((aTSC > bTSC) ? (aTSC - bTSC) : 0);

    return (1. - (idle_flits / tsc));
}

/*! \brief Get estimation of QPI (data+nondata) traffic per outgoing QPI link

    Returns an estimation of number of data bytes transferred from a socket over Intel(r) Quick Path Interconnect

    \param socketNr socket identifier
    \param linkNr linkNr
    \param before System CPU counter state before the experiment
    \param after System CPU counter state after the experiment
    \return Number of bytes
*/
inline uint64 getOutgoingQPILinkBytes(uint32 socketNr, uint32 linkNr, const SystemCounterState & before, const SystemCounterState & after)
{
    PCM * m = PCM::getInstance();
    const int cpu_model = m->getCPUModel();
    if (cpu_model != PCM::NEHALEM_EX && cpu_model != PCM::WESTMERE_EX) return 0;

    const double util = getOutgoingQPILinkUtilization(socketNr, linkNr, before, after);
    const double max_bytes = (double(m->getQPILinkSpeed()) * double(getInvariantTSC(before, after) / double(m->getNumCores())) / double(m->getNominalFrequency()));

    return (uint64)(max_bytes * util);
}


/*! \brief Get estimation of total QPI data traffic

    Returns an estimation of number of data bytes transferred to all sockets over all Intel(r) Quick Path Interconnect links

    \param before System CPU counter state before the experiment
    \param after System CPU counter state after the experiment
    \return Number of bytes
*/
inline uint64 getAllIncomingQPILinkBytes(const SystemCounterState & before, const SystemCounterState & after)
{
    PCM * m = PCM::getInstance();
    const uint32 ns = m->getNumSockets();
    const uint32 qpiLinks = (uint32)m->getQPILinksPerSocket();
    uint64 sum = 0;

    for (uint32 s = 0; s < ns; ++s)
        for (uint32 q = 0; q < qpiLinks; ++q)
            sum += getIncomingQPILinkBytes(s, q, before, after);

    return sum;
}

/*! \brief Get estimation of total QPI data+nondata traffic

    Returns an estimation of number of data and non-data bytes transferred from all sockets over all Intel(r) Quick Path Interconnect links

    \param before System CPU counter state before the experiment
    \param after System CPU counter state after the experiment
    \return Number of bytes
*/
inline uint64 getAllOutgoingQPILinkBytes(const SystemCounterState & before, const SystemCounterState & after)
{
    PCM * m = PCM::getInstance();
    const uint32 ns = m->getNumSockets();
    const uint32 qpiLinks = (uint32)m->getQPILinksPerSocket();
    uint64 sum = 0;

    for (uint32 s = 0; s < ns; ++s)
        for (uint32 q = 0; q < qpiLinks; ++q)
            sum += getOutgoingQPILinkBytes(s, q, before, after);

    return sum;
}


/*! \brief Return current value of the counter of QPI data traffic per incoming QPI link

    Returns the number of incoming data bytes to a socket over Intel(r) Quick Path Interconnect

    \param socketNr socket identifier
    \param linkNr linkNr
    \param now Current System CPU counter state
    \return Number of bytes
*/
inline uint64 getIncomingQPILinkBytes(uint32 socketNr, uint32 linkNr, const SystemCounterState & now)
{
    return 64 * now.incomingQPIPackets[socketNr][linkNr];
}

/*! \brief Get estimation of total QPI data traffic for this socket

    Returns an estimation of number of bytes transferred to this sockets over all Intel(r) Quick Path Interconnect links on this socket

    \param before System CPU counter state before the experiment
    \param after System CPU counter state after the experiment
    \return Number of bytes
*/
inline uint64 getSocketIncomingQPILinkBytes(uint32 socketNr, const SystemCounterState & now)
{
    PCM * m = PCM::getInstance();
    const uint32 qpiLinks = (uint32)m->getQPILinksPerSocket();
    uint64 sum = 0;

    for (uint32 q = 0; q < qpiLinks; ++q)
        sum += getIncomingQPILinkBytes(socketNr, q, now);

    return sum;
}

/*! \brief Get estimation of Socket QPI data traffic

    Returns an estimation of number of data bytes transferred to all sockets over all Intel(r) Quick Path Interconnect links

    \param now System CPU counter state
    \return Number of bytes
*/
inline uint64 getAllIncomingQPILinkBytes(const SystemCounterState & now)
{
    PCM * m = PCM::getInstance();
    const uint32 ns = m->getNumSockets();
    uint64 sum = 0;

    for (uint32 s = 0; s < ns; ++s)
        sum += getSocketIncomingQPILinkBytes(s, now);
    return sum;
}


/*! \brief Get QPI data to Memory Controller traffic ratio

    Ideally for NUMA-optmized programs the ratio should be close to 0.

    \param before System CPU counter state before the experiment
    \param after System CPU counter state after the experiment
    \return Ratio
*/

inline double getQPItoMCTrafficRatio(const SystemCounterState & before, const SystemCounterState & after)
{
    const uint64 totalQPI = getAllIncomingQPILinkBytes(before, after);
    const uint64 memTraffic = getBytesReadFromMC(before, after) + getBytesWrittenToMC(before, after);
    return double(totalQPI) / double(memTraffic);
}

#endif
