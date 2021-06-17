/* 
* CTaskManager.h
*
* Created: 04/11/2017 10:57:36
* Author: phil
*/


#ifndef __CTASKMANAGER_H__
#define __CTASKMANAGER_H__
//#include <functional>

#define LOAD_PERCENTAGE_ONE 100
#define MOVING_SUM_COUNT 32

#define SCHEDULER_DELAY_LIMIT 1


typedef uint32_t TimeUs_t;

typedef uint32_t TimeDelta_t;					// time difference, 32 bits always sufficient
#define TASK_PERIOD_HZ(hz) (1000000 / (hz))
#define TASK_PERIOD_MS(ms) ((ms) * 1000)
#define TASK_PERIOD_US(us) (us)


//static inline TimeDelta_t cmpTimeUs(TimeUs_t a, TimeUs_t b) { return (TimeDelta_t)(a - b); }




typedef enum 
{
	TASK_PRIORITY_IDLE			= 0,     // Disables dynamic scheduling, task is executed only if no other task is active this cycle
	TASK_PRIORITY_LOW			= 1,
	TASK_PRIORITY_MEDIUM		= 3,
	TASK_PRIORITY_MEDIUM_HIGH	= 4,
	TASK_PRIORITY_HIGH			= 5,
	TASK_PRIORITY_REALTIME		= 6,
	TASK_PRIORITY_MAX			= 255
} TaskPriority_t;

// typedef struct 
// {
// 	TimeUs_t     maxExecutionTime;
// 	TimeUs_t     totalExecutionTime;
// 	TimeUs_t     averageExecutionTime;
// } CheckFuncInfo_t;


typedef enum 
{
	/* Actual tasks */
	TASK_TASK_SYSTEM =0,
	TASK_SYSTEM,
	TASK_CHECK_ALARMS,
	TASK_RADIO_VALUES ,
	TASK_UPDATE_GPS,
	TASK_UPDATE_ATTITUDE,
	TASK_UPDATE_AHRS,
	TASK_UPDATE_BEARINGS,
	TASK_NAVIGATION_UPDATE,
	TASK_UPDATE_GUI,
	TASK_UPDATE_DEBUG_DISPLAY,
	TASK_UPDATE_SENSORS,
	TASK_UPDATE_TRIP_VARS,
	TASK_UPDATE_REPORT_STATUS,
	TASK_UPDATE_MOTOR,
	TASK_IMU_UPDATE,
	TASK_DELAYED_SETUP,
	TASK_UPDATE_OSD,
	TASK_AVOIDANCE,
	TASK_FRSKY_TELE_UPDATE,
	TASK_HEAD_UPDATE,
	TASK_FRONT_SCAN,
	TASK_PATH_PLANNER,
	TASK_FRONT_RANGE,
	TASK_WHEEL_ENCODERS,
	TASK_START_TASKS,
	TASK_UPDATE_SOUND,
	TASK_COUNT,					// Task Count must be at end of tasks
	NOT_IN_QUE

} TaskId_t;

typedef struct
{
	TaskId_t	TaskId;
	char		Lable[15];
	bool		isEnabled;
	uint8_t		TaskPriority;
	TimeDelta_t	DesiredPeriod;
	TimeDelta_t	ActualPeriod;
	TimeUs_t	maxExecutionTime;
	TimeUs_t	averageExecutionTime;
	uint16_t	SystemLoad;
	uint16_t	QueSize;  
} TaskInfo_t;



typedef struct // __attribute__ ((packed))
{
	// Configuration
	char			Lable[15];
	TaskId_t		TaskId;
	TimeDelta_t		DesiredPeriod;							// target period of execution
	uint8_t			TaskPriority;							// dynamicPriority grows in steps of this size, shouldn't be zero
	bool			IsEventDriven;
	// Scheduling
	uint16_t		DynamicPriority;						// measurement of how old task was last executed, used to avoid task starvation
	uint16_t		TaskAgeCycles;
	uint32_t		ActualPeriod;
	TimeUs_t		LastExecutedAt;							// last time of invocation
	TimeUs_t		lastSignaledAt;							// time of invocation event for event-driven tasks

	// Statistics
	TimeUs_t		AverageExecutionTime;					// moving sum over 32 samples
	TimeUs_t		maxExecutionTime;
	uint8_t			AverageCount;
} Task_t;


class CTaskManager
{
//variables
public:
	uint16_t m_AverageSystemLoadPercent;
	uint32_t m_TotalWaitingTasks;
	uint32_t m_TotalWaitingTasksSamples;
	uint16_t m_TaskQueueSize = 0;
	uint16_t m_TaskQueuePos = 0;
	bool m_CalculateTaskStatistics;
	Task_t* m_TaskQueueArray[TASK_COUNT + 1]; // extra item for NULL pointer at end of queue
	Task_t m_Tasks[TASK_COUNT+1];
protected:
private:
	uint32_t m_TaskExecutionTime;
//functions
public:
	CTaskManager();
	~CTaskManager();
	void SystemTask(TimeUs_t currentTimeUs);
	void ClearQueue(void);
	bool IsTaskInQue(TaskId_t TaskId);
	bool AddToQue(Task_t *task);
	bool RemoveFromQue(Task_t *task);
	Task_t *GetFirstInQueue(void);
	Task_t *GetNextInQueue(void);
	bool GetTimedTaskInfo(TaskId_t taskId, TaskInfo_t *taskInfo);
	bool GetTimedTaskInfo(uint16_t QuePosition, TaskInfo_t * taskInfo);
	void RescheduleTask(TaskId_t taskId, uint32_t newPeriodMicros);
	void EnableTask(TaskId_t taskId, bool newEnabledState);
	TimeDelta_t GetTaskDeltaTime(TaskId_t taskId);
	void SetCalulateTaskStatistics(bool calculateTaskStatistics);
	void ResetTaskStatistics(TaskId_t taskId);

	void Init(void);
	void Scheduler(void);
	void DoTasks(TaskId_t Task,uint32_t Time);
	void Delay(uint32_t ms);
	void Delay(uint8_t Delay);
	void GetTaskInfo(uint8_t TaskNo, TaskInfo_t * taskInfo);
	bool GetTaskInfo(TaskId_t TaskNo, TaskInfo_t * taskInfo);
	bool CheckEventTrigger(TaskId_t Task,TimeUs_t LastTime,TimeUs_t NowTime);
	bool isSystemOverloaded(){return(m_AverageSystemLoadPercent >= LOAD_PERCENTAGE_ONE);};
protected:
private:
	CTaskManager( const CTaskManager &c );
	CTaskManager& operator=( const CTaskManager &c );

}; //CTaskManager





#endif //__CTASKMANAGER_H__
