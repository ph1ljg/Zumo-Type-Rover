/* 
* CTaskManager.cpp
*
* Created: 04/11/2017 10:57:36
* Author: phil
*/

#include "Includes.h"


uint32_t TaskTime;
uint32_t NowTimeUs;


// default constructor
CTaskManager::CTaskManager()
{

} //CTaskManager

// default destructor
CTaskManager::~CTaskManager()
{
} //~CTaskManager


void CTaskManager::Init(void)
{
	m_CalculateTaskStatistics = true;

	unsigned char i;
	m_AverageSystemLoadPercent = 0;
	strncpy((char*)(m_Tasks[TASK_TASK_SYSTEM].Lable),"System",7);
	m_Tasks[TASK_TASK_SYSTEM].TaskId							= TASK_TASK_SYSTEM;
	m_Tasks[TASK_TASK_SYSTEM].DesiredPeriod					= TASK_PERIOD_HZ(10);        // 10Hz, every 100 ms
	m_Tasks[TASK_TASK_SYSTEM].TaskPriority					= TASK_PRIORITY_MEDIUM_HIGH;
	m_Tasks[TASK_TASK_SYSTEM].IsEventDriven					= false;

	strncpy((char*)(m_Tasks[TASK_SYSTEM].Lable),"System",7);
	m_Tasks[TASK_SYSTEM].TaskId							= TASK_TASK_SYSTEM;
	m_Tasks[TASK_SYSTEM].DesiredPeriod					= TASK_PERIOD_MS(200);        
	m_Tasks[TASK_SYSTEM].TaskPriority					= TASK_PRIORITY_MEDIUM_HIGH;
	m_Tasks[TASK_SYSTEM].IsEventDriven					= false;


	strncpy((char*)(m_Tasks[TASK_RADIO_VALUES].Lable),"Update Radio",13);
	m_Tasks[TASK_RADIO_VALUES].TaskId					=TASK_RADIO_VALUES;
	m_Tasks[TASK_RADIO_VALUES].DesiredPeriod			= TASK_PERIOD_MS(7);
	m_Tasks[TASK_RADIO_VALUES].TaskPriority				= TASK_PRIORITY_REALTIME;
	m_Tasks[TASK_RADIO_VALUES].IsEventDriven			= false;

	strncpy((char*)(m_Tasks[TASK_UPDATE_REPORT_STATUS].Lable),"Flash Leds",11);
	m_Tasks[TASK_UPDATE_REPORT_STATUS].TaskId				= TASK_UPDATE_REPORT_STATUS;
	m_Tasks[TASK_UPDATE_REPORT_STATUS].DesiredPeriod		= TASK_PERIOD_MS(1000);
	m_Tasks[TASK_UPDATE_REPORT_STATUS].TaskPriority		= TASK_PRIORITY_LOW;
	m_Tasks[TASK_UPDATE_REPORT_STATUS].IsEventDriven		= false;

	strncpy((char*)(m_Tasks[TASK_FRONT_SCAN].Lable),"Front Scan",11);
	m_Tasks[TASK_FRONT_SCAN].TaskId				= TASK_FRONT_SCAN;
	m_Tasks[TASK_FRONT_SCAN].DesiredPeriod		= TASK_PERIOD_MS(10);
	m_Tasks[TASK_FRONT_SCAN].TaskPriority		= TASK_PRIORITY_LOW;
	m_Tasks[TASK_FRONT_SCAN].IsEventDriven		= false;

// 	strncpy((char*)(m_Tasks[TASK_FRONT_RANGE].Lable),"Front Rng",10);
// 	m_Tasks[TASK_FRONT_RANGE].TaskId				= TASK_FRONT_RANGE;
// 	m_Tasks[TASK_FRONT_RANGE].DesiredPeriod		= TASK_PERIOD_MS(500);
// 	m_Tasks[TASK_FRONT_RANGE].TaskPriority		= TASK_PRIORITY_MEDIUM;
// 	m_Tasks[TASK_FRONT_RANGE].IsEventDriven		= false;



	strncpy((char*)m_Tasks[TASK_NAVIGATION_UPDATE].Lable,"Navigation",11);
	m_Tasks[TASK_NAVIGATION_UPDATE].TaskId				= TASK_NAVIGATION_UPDATE;
	m_Tasks[TASK_NAVIGATION_UPDATE].DesiredPeriod		= TASK_PERIOD_MS(20);
	m_Tasks[TASK_NAVIGATION_UPDATE].TaskPriority		= TASK_PRIORITY_HIGH;
	m_Tasks[TASK_NAVIGATION_UPDATE].IsEventDriven		= false;

	strncpy((char*)m_Tasks[TASK_UPDATE_ATTITUDE].Lable,"Attitude Update",16);
	m_Tasks[TASK_UPDATE_ATTITUDE].TaskId						= TASK_UPDATE_ATTITUDE;
	m_Tasks[TASK_UPDATE_ATTITUDE].DesiredPeriod				= TASK_PERIOD_MS(60);
	m_Tasks[TASK_UPDATE_ATTITUDE].TaskPriority				= TASK_PRIORITY_MEDIUM_HIGH;
	m_Tasks[TASK_UPDATE_ATTITUDE].IsEventDriven				= false;

	strncpy((char*)m_Tasks[TASK_UPDATE_AHRS].Lable,"Imu update",11);
	m_Tasks[TASK_UPDATE_AHRS].TaskId						= TASK_UPDATE_AHRS;
	m_Tasks[TASK_UPDATE_AHRS].DesiredPeriod				= TASK_PERIOD_MS(50);
	m_Tasks[TASK_UPDATE_AHRS].TaskPriority				= TASK_PRIORITY_HIGH;
	m_Tasks[TASK_UPDATE_AHRS].IsEventDriven				= false;



	strncpy((char*)m_Tasks[TASK_UPDATE_GUI].Lable,"GUI Update",11);
	m_Tasks[TASK_UPDATE_GUI].TaskId						= TASK_UPDATE_GUI;
	m_Tasks[TASK_UPDATE_GUI].DesiredPeriod				= TASK_PERIOD_MS(10);
	m_Tasks[TASK_UPDATE_GUI].TaskPriority				= TASK_PRIORITY_HIGH;
	m_Tasks[TASK_UPDATE_GUI].IsEventDriven				= false;
	
	strncpy((char*)m_Tasks[TASK_IMU_UPDATE].Lable,"Imu Update",12);
	m_Tasks[TASK_IMU_UPDATE].TaskId			= TASK_IMU_UPDATE;
	m_Tasks[TASK_IMU_UPDATE].DesiredPeriod		= TASK_PERIOD_MS(20);
	m_Tasks[TASK_IMU_UPDATE].TaskPriority		= TASK_PRIORITY_MEDIUM;
	m_Tasks[TASK_IMU_UPDATE].IsEventDriven		= false;



	strncpy((char*)m_Tasks[TASK_UPDATE_SENSORS].Lable,"Battery",8);
	m_Tasks[TASK_UPDATE_SENSORS].TaskId			= TASK_UPDATE_SENSORS;
	m_Tasks[TASK_UPDATE_SENSORS].DesiredPeriod	= TASK_PERIOD_MS(1000);
	m_Tasks[TASK_UPDATE_SENSORS].TaskPriority	= TASK_PRIORITY_MEDIUM;
	m_Tasks[TASK_UPDATE_SENSORS].IsEventDriven	= false;

	strncpy((char*)m_Tasks[TASK_CHECK_ALARMS].Lable,"Alarms",7);
	m_Tasks[TASK_CHECK_ALARMS].TaskId					= TASK_CHECK_ALARMS;
	m_Tasks[TASK_CHECK_ALARMS].DesiredPeriod			= TASK_PERIOD_MS(600);
	m_Tasks[TASK_CHECK_ALARMS].TaskPriority				= TASK_PRIORITY_MEDIUM;
	m_Tasks[TASK_CHECK_ALARMS].IsEventDriven			= false;

	strncpy((char*)m_Tasks[TASK_UPDATE_GPS].Lable,"Gps",4);
	m_Tasks[TASK_UPDATE_GPS].TaskId						= TASK_UPDATE_GPS;
	m_Tasks[TASK_UPDATE_GPS].DesiredPeriod				= TASK_PERIOD_MS(100);
	m_Tasks[TASK_UPDATE_GPS].TaskPriority				= TASK_PRIORITY_MEDIUM;
	m_Tasks[TASK_UPDATE_GPS].IsEventDriven				= false;


	strncpy((char*)m_Tasks[TASK_UPDATE_DEBUG_DISPLAY].Lable,"Ext Display",12);
	m_Tasks[TASK_UPDATE_DEBUG_DISPLAY].TaskId			= TASK_UPDATE_DEBUG_DISPLAY;
	m_Tasks[TASK_UPDATE_DEBUG_DISPLAY].DesiredPeriod	= TASK_PERIOD_MS(800);
	m_Tasks[TASK_UPDATE_DEBUG_DISPLAY].TaskPriority		= TASK_PRIORITY_LOW;
	m_Tasks[TASK_UPDATE_DEBUG_DISPLAY].IsEventDriven	= true;


	strncpy(m_Tasks[TASK_UPDATE_BEARINGS].Lable,"Nav Processor",14);
	m_Tasks[TASK_UPDATE_BEARINGS].TaskId				= TASK_UPDATE_BEARINGS;
	m_Tasks[TASK_UPDATE_BEARINGS].DesiredPeriod			= TASK_PERIOD_MS(1000);
	m_Tasks[TASK_UPDATE_BEARINGS].TaskPriority			= TASK_PRIORITY_MEDIUM_HIGH;
	m_Tasks[TASK_UPDATE_BEARINGS].IsEventDriven			= false;


	strncpy((char*)m_Tasks[TASK_HEAD_UPDATE].Lable,"Head",5);
	m_Tasks[TASK_HEAD_UPDATE].TaskId					= TASK_HEAD_UPDATE;
	m_Tasks[TASK_HEAD_UPDATE].DesiredPeriod				= TASK_PERIOD_MS(60);
	m_Tasks[TASK_HEAD_UPDATE].TaskPriority				= TASK_PRIORITY_HIGH;
	m_Tasks[TASK_HEAD_UPDATE].IsEventDriven				= false;
	
	strncpy((char*)(m_Tasks[TASK_AVOIDANCE].Lable),"Obstacle Avoid",15);
	m_Tasks[TASK_AVOIDANCE].TaskId				= TASK_AVOIDANCE;
	m_Tasks[TASK_AVOIDANCE].DesiredPeriod		= TASK_PERIOD_MS(200);
	m_Tasks[TASK_AVOIDANCE].TaskPriority		= TASK_PRIORITY_HIGH;
	m_Tasks[TASK_AVOIDANCE].IsEventDriven		= false;

	// 	strncpy((char*)m_Tasks[TASK_CALIBRATE].Lable,"Calibrate",9);
	// 	m_Tasks[TASK_CALIBRATE].TaskId						= TASK_CALIBRATE;

	// 	m_Tasks[TASK_CALIBRATE].DesiredPeriod				= TASK_PERIOD_MS(85);
	// 	m_Tasks[TASK_CALIBRATE].TaskPriority				= TASK_PRIORITY_HIGH;
	// 	m_Tasks[TASK_CALIBRATE].IsEventDriven				= false;




	strncpy((char*)m_Tasks[TASK_UPDATE_MOTOR].Lable,"Motor",6);
	m_Tasks[TASK_UPDATE_MOTOR].TaskId					= TASK_UPDATE_MOTOR;
	m_Tasks[TASK_UPDATE_MOTOR].DesiredPeriod			= TASK_PERIOD_MS(20);
	m_Tasks[TASK_UPDATE_MOTOR].TaskPriority				= TASK_PRIORITY_MEDIUM;
	m_Tasks[TASK_UPDATE_MOTOR].IsEventDriven			= false;

	strncpy((char*)m_Tasks[TASK_UPDATE_OSD].Lable,"Osd",4);
	m_Tasks[TASK_UPDATE_OSD].TaskId						= TASK_UPDATE_OSD;
	m_Tasks[TASK_UPDATE_OSD].DesiredPeriod				= TASK_PERIOD_MS(100);
	m_Tasks[TASK_UPDATE_OSD].TaskPriority				= TASK_PRIORITY_HIGH;
	m_Tasks[TASK_UPDATE_OSD].IsEventDriven				= false;

	strncpy((char*)m_Tasks[TASK_UPDATE_TRIP_VARS].Lable,"Trip Vars",10);
	m_Tasks[TASK_UPDATE_TRIP_VARS].TaskId				= TASK_UPDATE_TRIP_VARS;
	m_Tasks[TASK_UPDATE_TRIP_VARS].DesiredPeriod		= TASK_PERIOD_MS(1000);
	m_Tasks[TASK_UPDATE_TRIP_VARS].TaskPriority			= TASK_PRIORITY_HIGH;
	m_Tasks[TASK_UPDATE_TRIP_VARS].IsEventDriven		= false;

	strncpy((char*)m_Tasks[TASK_DELAYED_SETUP].Lable,"Delayed Setup",14);
	m_Tasks[TASK_DELAYED_SETUP].TaskId					= TASK_DELAYED_SETUP;
	m_Tasks[TASK_DELAYED_SETUP].DesiredPeriod			= TASK_PERIOD_MS(8000);
	m_Tasks[TASK_DELAYED_SETUP].TaskPriority			= TASK_PRIORITY_MEDIUM;
	m_Tasks[TASK_DELAYED_SETUP].IsEventDriven			= false;



	strncpy((char*)m_Tasks[TASK_FRSKY_TELE_UPDATE].Lable,"FrSky Update",13);
	m_Tasks[TASK_FRSKY_TELE_UPDATE].TaskId				= TASK_FRSKY_TELE_UPDATE;
	m_Tasks[TASK_FRSKY_TELE_UPDATE].DesiredPeriod		= TASK_PERIOD_MS(1);
	m_Tasks[TASK_FRSKY_TELE_UPDATE].TaskPriority		= TASK_PRIORITY_HIGH;
	m_Tasks[TASK_FRSKY_TELE_UPDATE].IsEventDriven		= false;

	strncpy((char*)m_Tasks[TASK_PATH_PLANNER].Lable,"Planner",8);
	m_Tasks[TASK_PATH_PLANNER].TaskId					= TASK_PATH_PLANNER;
	m_Tasks[TASK_PATH_PLANNER].DesiredPeriod				= TASK_PERIOD_MS(1000);
	m_Tasks[TASK_PATH_PLANNER].TaskPriority				= TASK_PRIORITY_MEDIUM;
	m_Tasks[TASK_PATH_PLANNER].IsEventDriven				= false;

	strncpy((char*)m_Tasks[TASK_WHEEL_ENCODERS].Lable,"Wheel Enc",10);
	m_Tasks[TASK_WHEEL_ENCODERS].TaskId					= TASK_WHEEL_ENCODERS;
	m_Tasks[TASK_WHEEL_ENCODERS].DesiredPeriod			= TASK_PERIOD_MS(40);
	m_Tasks[TASK_WHEEL_ENCODERS].TaskPriority			= TASK_PRIORITY_HIGH;
	m_Tasks[TASK_WHEEL_ENCODERS].IsEventDriven			= false;



	strncpy((char*)m_Tasks[TASK_START_TASKS].Lable,"Start Tasks",12);
	m_Tasks[TASK_START_TASKS].TaskId					= TASK_START_TASKS;
	m_Tasks[TASK_START_TASKS].DesiredPeriod				= TASK_PERIOD_MS(20);
	m_Tasks[TASK_START_TASKS].TaskPriority				= TASK_PRIORITY_HIGH;
	m_Tasks[TASK_START_TASKS].IsEventDriven				= false;



	strncpy((char*)m_Tasks[TASK_UPDATE_SOUND].Lable,"Sound",5);
	m_Tasks[TASK_UPDATE_SOUND].TaskId					= TASK_UPDATE_SOUND;
	m_Tasks[TASK_UPDATE_SOUND].DesiredPeriod			= TASK_PERIOD_MS(20);
	m_Tasks[TASK_UPDATE_SOUND].TaskPriority				= TASK_PRIORITY_MEDIUM;
	m_Tasks[TASK_UPDATE_SOUND].IsEventDriven			= false;



	for(i=0;i<TASK_COUNT;i++)
	{
		m_Tasks[i].maxExecutionTime = 0;
		m_Tasks[i].AverageExecutionTime = 0;
		m_Tasks[i].ActualPeriod = 0;
		m_Tasks[i].DynamicPriority = 0;
		m_Tasks[i].LastExecutedAt = 0;
		m_Tasks[i].lastSignaledAt = 0;
		m_Tasks[i].TaskAgeCycles = 0;
		m_Tasks[i].AverageCount = 1;
		

	}
	ClearQueue();
	//AddToQue(&m_Tasks[TASK_SYSTEM]);
}

void CTaskManager::SystemTask(TimeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);

	// Calculate system load
	if (m_TotalWaitingTasksSamples > 0) 
	{
		m_AverageSystemLoadPercent = 100 * m_TotalWaitingTasks / m_TotalWaitingTasksSamples;
		m_TotalWaitingTasksSamples = 0;
		m_TotalWaitingTasks = 0;
	}
}






void CTaskManager::ClearQueue(void)
{
    memset(m_TaskQueueArray, 0, sizeof(m_TaskQueueArray));
    m_TaskQueuePos = 0;
    m_TaskQueueSize = 0;
}

bool CTaskManager::IsTaskInQue(TaskId_t TaskId)
{
    for (int ii = 0; ii < m_TaskQueueSize; ++ii) 
	{
        if (m_TaskQueueArray[ii]->TaskId == TaskId) 
		{
            return true;
        }
    }
    return false;
}

bool CTaskManager::AddToQue(Task_t *task)
{
    if ((m_TaskQueueSize >= TASK_COUNT) || IsTaskInQue(task->TaskId)) 
	{
        return false;
    }
	task->LastExecutedAt = Core.micros();					// set for start of period
    for (int ii = 0; ii <= m_TaskQueueSize; ++ii) 
	{
        if (m_TaskQueueArray[ii] == NULL || m_TaskQueueArray[ii]->TaskPriority < task->TaskPriority) 
		{
            memmove(&m_TaskQueueArray[ii+1], &m_TaskQueueArray[ii], sizeof(task) * (m_TaskQueueSize - ii));
            m_TaskQueueArray[ii] = task;
            ++m_TaskQueueSize;
            return true;
        }
    }
    return false;
}

bool CTaskManager::RemoveFromQue(Task_t *task)
{
    for (int ii = 0; ii < m_TaskQueueSize; ++ii) 
	{
        if (m_TaskQueueArray[ii]->TaskId == task->TaskId) 
		{
            memmove(&m_TaskQueueArray[ii], &m_TaskQueueArray[ii+1], sizeof(task) * (m_TaskQueueSize - ii));
            --m_TaskQueueSize;
            return true;
        }
    }
    return false;
}

//================================================================================
// Returns first item queue or NULL if queue empty
//================================================================================
Task_t *CTaskManager::GetFirstInQueue(void)
{
    m_TaskQueuePos = 0;
    return m_TaskQueueArray[0]; // guaranteed to be NULL if queue is empty
}

//================================================================================
// Returns next item in queue or NULL if at end of queue
//================================================================================

Task_t *CTaskManager::GetNextInQueue(void)
{
    
	return m_TaskQueueArray[++m_TaskQueuePos]; // guaranteed to be NULL at end of queue
}

bool CTaskManager::GetTimedTaskInfo(uint16_t QuePosition, TaskInfo_t * taskInfo)
{
	if( QuePosition > m_TaskQueueSize)
		return(false);
	strcpy((char*)taskInfo->Lable,m_TaskQueueArray[QuePosition]->Lable);
    taskInfo->TaskId				= m_TaskQueueArray[QuePosition]->TaskId;
	taskInfo->isEnabled				= true;
    taskInfo->DesiredPeriod			= m_TaskQueueArray[QuePosition]->DesiredPeriod;
    taskInfo->TaskPriority			= m_TaskQueueArray[QuePosition]->TaskPriority;
    taskInfo->maxExecutionTime		= m_TaskQueueArray[QuePosition]->maxExecutionTime;
    taskInfo->averageExecutionTime	= m_TaskQueueArray[QuePosition]->AverageExecutionTime / MOVING_SUM_COUNT;
    taskInfo->ActualPeriod			= m_TaskQueueArray[QuePosition]->ActualPeriod;
	return(true);
}



bool CTaskManager::GetTimedTaskInfo(TaskId_t TaskId, TaskInfo_t * taskInfo)
{
	if( !IsTaskInQue(TaskId))
		return(false);

    taskInfo->TaskId				= m_Tasks[TaskId].TaskId;
	taskInfo->isEnabled				= true;
    taskInfo->DesiredPeriod			= m_Tasks[TaskId].DesiredPeriod;
    taskInfo->TaskPriority			= m_Tasks[TaskId].TaskPriority;
    taskInfo->maxExecutionTime		= m_Tasks[TaskId].maxExecutionTime;
    taskInfo->averageExecutionTime	= m_Tasks[TaskId].AverageExecutionTime / MOVING_SUM_COUNT;
    taskInfo->ActualPeriod			= m_Tasks[TaskId].ActualPeriod;
	return(true);
}

void CTaskManager::RescheduleTask(TaskId_t TaskId, uint32_t NewPeriodMicros)
{
 	if (TaskId < TASK_COUNT) 
	{
        Task_t *Task = &m_Tasks[TaskId];
        Task->DesiredPeriod = max(SCHEDULER_DELAY_LIMIT, (TimeDelta_t)NewPeriodMicros);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    }
}

void CTaskManager::EnableTask(TaskId_t TaskId, bool Enabled)
{
    if (TaskId < TASK_COUNT) 
	{
        Task_t *Task = &m_Tasks[TaskId];
         if (Enabled ) 
             AddToQue(Task);
 		else 
             RemoveFromQue(Task);
    }
}

TimeDelta_t CTaskManager::GetTaskDeltaTime(TaskId_t TaskId)
{
	if (TaskId < TASK_COUNT) 
        return m_Tasks[TaskId].ActualPeriod;
	else 
        return 0;
}

void CTaskManager::SetCalulateTaskStatistics(bool calculateTaskStatisticsToUse)
{
    m_CalculateTaskStatistics = calculateTaskStatisticsToUse;
}

void CTaskManager::ResetTaskStatistics(TaskId_t taskId)
{
#ifdef SKIP_TASK_STATISTICS
    UNUSED(taskId);
#else
	if (taskId < TASK_COUNT) 
	{
        m_Tasks[taskId].AverageExecutionTime = 0;
    }
#endif
}


void CTaskManager::Scheduler(void)
{
   
	uint16_t selectedTaskDynamicPriority = 0;
    uint16_t WaitingTasks = 0;
    Task_t *SelectedTask = NULL;									// The task to be invoked
	TimeUs_t TimeBeforeEventCheck;
	
	Task_t *Task;
	NowTimeUs = Core.micros();						// Cache currentTime
    
    for (Task = GetFirstInQueue(); Task != NULL; Task = GetNextInQueue())	// Update task dynamic priorities
	{
		if (Task->IsEventDriven)													// Event driven 
		{
            TimeBeforeEventCheck = NowTimeUs;
            
            if (Task->DynamicPriority > 0)											// Increase priority for event driven tasks
			{
                Task->TaskAgeCycles = 1 + ((NowTimeUs - Task->lastSignaledAt) / Task->DesiredPeriod);
                Task->DynamicPriority = 1 + Task->TaskPriority * Task->TaskAgeCycles;
                WaitingTasks++;
            } 
			else if(CheckEventTrigger(Task->TaskId,TimeBeforeEventCheck, TimeBeforeEventCheck - Task->LastExecutedAt)) 
			{
                Task->lastSignaledAt = TimeBeforeEventCheck;
                Task->TaskAgeCycles = 1;
                Task->DynamicPriority = 1 + Task->TaskPriority;
                WaitingTasks++;
            } 
			else 
                Task->TaskAgeCycles = 0;
        } 
		else 
		{
			TaskTime = (NowTimeUs - Task->LastExecutedAt);
            Task->TaskAgeCycles = (uint16_t)( TaskTime/ Task->DesiredPeriod);		// Task is time-driven, dynamicPriority is last execution age (measured in desiredPeriods)
            if (Task->TaskAgeCycles > 0)																	// Task age is calculated from last execution
			{
                Task->DynamicPriority = 1 + Task->TaskPriority * Task->TaskAgeCycles;
                WaitingTasks++;
            }
        }
	    if (Task->DynamicPriority > selectedTaskDynamicPriority) 
		{
			if (Task->TaskAgeCycles > 0) 
			{
				selectedTaskDynamicPriority = Task->DynamicPriority;
				SelectedTask = Task;
			}
		}
	}

    m_TotalWaitingTasksSamples++;
    m_TotalWaitingTasks += WaitingTasks;

    if (SelectedTask) 
	{
		if(SelectedTask->LastExecutedAt != 0)
		{
	        TaskTime =  NowTimeUs - SelectedTask->LastExecutedAt;
			
			if(TaskTime >SelectedTask->DesiredPeriod)
			{
				SelectedTask->ActualPeriod = TaskTime;	// Found a task that should be run
				if (m_CalculateTaskStatistics) // Execute task
				{
					TimeUs_t TimeBeforeTask = Core.micros();
					DoTasks( SelectedTask->TaskId,TaskTime);
					NowTimeUs = Core.micros();
//					if(NowTimeUs >TimeBeforeTask)
						m_TaskExecutionTime =(uint32_t) (NowTimeUs - TimeBeforeTask);			// if the time is small 1-2 us timer can be delayed by interrupts
//					else
//						TaskExecutionTime = 0;	
					SelectedTask->AverageExecutionTime = (SelectedTask->AverageExecutionTime*SelectedTask->AverageCount)+m_TaskExecutionTime;
					SelectedTask->AverageExecutionTime /=  (++SelectedTask->AverageCount);
					if(SelectedTask->AverageCount > 10)
					{
						SelectedTask->AverageCount = 1;
					}

					SelectedTask->maxExecutionTime = max(SelectedTask->maxExecutionTime, m_TaskExecutionTime);
//					if(TaskExecutionTime >2000000)
//						printf("");
				} 
 				else 
				 {
					DoTasks( SelectedTask->TaskId,TaskTime);
				 }
				SelectedTask->LastExecutedAt = Core.micros();
				SelectedTask->DynamicPriority = 0;
			}
			
		}
		else
			SelectedTask->LastExecutedAt = Core.micros();

    }
	//WatchDog.Restart();
}



void CTaskManager::DoTasks(TaskId_t Task,uint32_t Time)
{
	switch(Task)
	{
	case TASK_TASK_SYSTEM:
		SystemTask(Time);
		break;
	case TASK_SYSTEM:
//		Zumo.UpdateSystem();
		break;
	case TASK_RADIO_VALUES:
		RadioControl.Update(Time);
		break;
	case TASK_UPDATE_MOTOR:
		Steering.Update(Time);
		break;
	case TASK_UPDATE_SENSORS:
		Sensors.UpDate();
		break;
	case TASK_UPDATE_REPORT_STATUS:
		StatusControl.Update();
		break;
	case TASK_FRSKY_TELE_UPDATE:
//		if(Config.m_RunningFlags.TELEMETRY_ACTIVE)
//			FrskyTelemetry.Update();
		break;
//	case TASK_FRONT_RANGE:
//		HeadControl.Update();
//		break;
	case TASK_START_TASKS:
		Zumo.StartTasks();
		break;
	case TASK_FRONT_SCAN:
		HeadControl.ScanFrontPath();
		break;
	case TASK_CHECK_ALARMS:
//		Alarms.Update();
		break;
	case TASK_UPDATE_ATTITUDE:
//		AttitudeCtl.Update();
		break;
	case TASK_UPDATE_AHRS:
		Ahrs.Update(Time);
		break;
	case TASK_UPDATE_GPS:
//		Gps.Update();
		break;
	case TASK_NAVIGATION_UPDATE:
//		Navigation.Update(Time);
		break;
	case TASK_UPDATE_BEARINGS:
//		Navigation.UpdateBearings();
		break;
//	case TASK_UPDATE_GUI_SETTINGS:
//			AutoBoat.CheckforGuiNavigationSettings();
//		break;
	case TASK_UPDATE_DEBUG_DISPLAY:
//		if(!Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY)
			DebugDisplay.DisplayLiveData();
		break;
	case TASK_UPDATE_TRIP_VARS:
//		Zumo.SetCourseDataVariables(Time);
	break;
	case TASK_UPDATE_GUI:
		GuiFunctions.SerialCom();
		break;
	case TASK_UPDATE_OSD:
		Osd.Update();
		break;
	case TASK_DELAYED_SETUP:
		Zumo.DelayedSetup();
		break;
	case TASK_AVOIDANCE:
		Avoidance.Update();
		break;
	case TASK_HEAD_UPDATE:
		HeadControl.Update();
		break;
	case TASK_WHEEL_ENCODERS:
		WheelEncoder.UpDate();
		break;
	case TASK_IMU_UPDATE:
		Imu.Update(Time);
		break;	
	case TASK_UPDATE_SOUND:
			ToneAlarm.Update();
	default:
		break;
	}

	
}



void CTaskManager::Delay(uint32_t ms)
{
	uint32_t start = Core.millis();

	while ((Core.millis() - start) < ms)
	{
		Scheduler();
		start += (m_TaskExecutionTime/1000);
	}
}


void CTaskManager::GetTaskInfo(uint8_t TaskNo, TaskInfo_t * taskInfo)
{
	switch(TaskNo)
	{
		case 1:
			GetTaskInfo(TASK_IMU_UPDATE,taskInfo);
			break;
		case 2:
			GetTaskInfo(TASK_RADIO_VALUES,taskInfo);
			break;
		case 3:
			GetTaskInfo(TASK_UPDATE_REPORT_STATUS,taskInfo);
			break;
		case 4:
			GetTaskInfo(TASK_UPDATE_GUI,taskInfo);
			break;
		case 5:
			GetTaskInfo(TASK_UPDATE_SENSORS,taskInfo);
			break;
		case 6:
			GetTaskInfo(TASK_UPDATE_AHRS,taskInfo);
			break;
		case 7:
			GetTaskInfo(TASK_NAVIGATION_UPDATE,taskInfo);
			break;
		case 8:
			GetTaskInfo(TASK_UPDATE_BEARINGS,taskInfo);
			break;
		case 9:
			GetTaskInfo(TASK_AVOIDANCE,taskInfo);
			break;
		case 10:
			GetTaskInfo(TASK_FRSKY_TELE_UPDATE,taskInfo);
			break;
		case 11:
			GetTaskInfo(TASK_UPDATE_OSD,taskInfo);
			break;
		case 12:
			GetTaskInfo(TASK_UPDATE_GPS,taskInfo);
			break;
		case 13:
			GetTaskInfo(TASK_UPDATE_ATTITUDE,taskInfo);
			break;
		case 14:
			GetTaskInfo(TASK_UPDATE_SENSORS,taskInfo);
			break;
	}

}


bool CTaskManager::GetTaskInfo(TaskId_t TaskNo, TaskInfo_t * taskInfo)
{

	if(!IsTaskInQue(TaskNo))
	{
		strncpy(taskInfo->Lable,m_Tasks[TaskNo].Lable,15);
		taskInfo->TaskId				= NOT_IN_QUE;
		taskInfo->DesiredPeriod			= 0;
		taskInfo->TaskPriority			= 0;
		taskInfo->maxExecutionTime		= 0;
		taskInfo->averageExecutionTime	= 0;
		taskInfo->ActualPeriod			= 0;
		taskInfo->SystemLoad			= 0;
		taskInfo->QueSize				= 0;
		return(false);
	}

	taskInfo->TaskId				= m_Tasks[TaskNo].TaskId;
	taskInfo->DesiredPeriod			= m_Tasks[TaskNo].DesiredPeriod;
	taskInfo->TaskPriority			= m_Tasks[TaskNo].TaskPriority;
	taskInfo->maxExecutionTime		= m_Tasks[TaskNo].maxExecutionTime;
	taskInfo->averageExecutionTime	= m_Tasks[TaskNo].AverageExecutionTime ;
	taskInfo->ActualPeriod			= m_Tasks[TaskNo].ActualPeriod;
	taskInfo->QueSize				= m_TaskQueueSize;
	taskInfo->SystemLoad			= m_AverageSystemLoadPercent;
	return(true);
}



bool CTaskManager::CheckEventTrigger(TaskId_t Task,TimeUs_t LastTime,TimeUs_t NowTime)
{
	 UNUSED(NowTime);
	 UNUSED(LastTime);
	bool RetVal = true;
	switch(Task)
	{
		case TASK_TASK_SYSTEM:
			break;
		default:
			break;		
	}
	return(RetVal);
}


void CTaskManager::Delay(uint8_t Delay)
{   uint32_t StartTime = Core.millis();
	while(1)
	{
		TaskManager.Scheduler();
		if(Core.millis()-StartTime >Delay)
			break;
	}
}