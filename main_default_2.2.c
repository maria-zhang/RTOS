
#include <LPC17xx.h>aqdf
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "context.h"

uint32_t msTicks = 0;
uint8_t bitVector = 0;
uint8_t numTasks = 0;

enum states{
		inactive,
		waiting,
		ready,
		running,
		blocked
} state;


enum priorities{ // priorities of different tasks
		idle,
		veryLow,
		low,
		normal,
		aboveNormal,
		thisIsImportant
} priority;


typedef struct TCB{
	uint8_t id;
	uint8_t priority;
	uint8_t sem; //semaphore flag
	uint32_t stack_top; //top of stack
	uint32_t stack_ptr; //current stack ptr (changes)
	uint8_t state;
	struct TCB * next;

}TCB_t;


//Variable Initializations for Semaphores
typedef struct {
	uint32_t semCount;
	TCB_t *waitList;	
}sem_t;

typedef struct {
	uint32_t muxCount;
	TCB_t *waitList;	
	uint8_t muxOwner;
}mux_t;

mux_t lock;
sem_t sem1;

typedef struct taskNode{
	TCB_t * head;
	uint8_t size;
	
}taskNode;

//declare priority queue
taskNode queue[6];



TCB_t * runningTask;

TCB_t tcbs[6];

void SysTick_Handler(void) {
    msTicks++;
	SCB->ICSR |=(1<<28);
}

void initalizeQueue(){
	for (int i = 0; i < 6; i++){
			queue[i].head = NULL;
			queue[i].size = 0;
		}
}

void initializeWaitList() {
	TCB_t waitList[6];
}

//Adds a TCB pointer to the tail of waitList
void addToWait(TCB_t tcbptr) {
	//traverse array to find first empty memory slot
	i = 0;
	while (waitList[i] == NULL) {
		i++;
	}
	
	//Store TCB pointer in empty array location
	waitList[i] = tcbptr;
}

TCB_t findWaitPriority() {
	TCB_t maxPriorityTask;
	maxPriorityTask.priority = 1;
	
	for(i=0; i++; i<6) {		
		if(waitList[i].priority > maxPriorityTask.priority) {
			maxPriorityTask.priority = waitList[i].priority;
		}
	}
	
	return maxPriorityTask;
}

//Removes the first element 
TCB_t removeFromWait() {
	//Remove TCB pointer in array location 0 by shifting all other memory locations to the left
	
	TCB_t removed = waitList[0];
	
	for(i=0; i++; i<6) {
		waitList[i] = waitList[i+1];
	}
	
	return removed;
}

void enqueue(TCB_t * newNode){

	uint8_t priority = newNode->priority;

	//check if first node 
	if (queue[priority].size == 0){
		queue[priority].head = newNode;
	}

	else {
		//iterate until next node is null
	 TCB_t * currNode = queue[priority].head;

		while (currNode->next != NULL){
			currNode = currNode->next;
		}
		currNode->next = newNode;
	}
	queue[priority].size++;
	newNode -> next = NULL;
	bitVector |= (1<<priority); 

}

TCB_t * dequeue(taskNode *list){  //returns pointer to deqeued node
	TCB_t * currNode = list->head;
	list->head = list->head->next;
	list->size--;
	if (!list->size)	
		bitVector &= ~(1<<currNode->priority);
	return currNode;
}



void initialization(){
	
	
	printf("initializing...\n" );
	
	uint32_t* vectorTable = 0X0;
	uint32_t mainStackAddress = vectorTable[0];
	
	for (int i = 0; i<6; i++){
		tcbs[i].stack_top = (mainStackAddress - 2048) - (1024*(5-i));
		tcbs[i].stack_ptr = (uint32_t *)address;
		tcbs[i].state = inactive;
		
	}
	
	uint32_t msp = __get_MSP();
	for (uint32_t i = 0; i < 1024; i++){
		*((uint32_t *)(tcbs[0].stack_top - i)) = *((uint32_t *)(mainStackAddress - i));
	}
	tcbs[0].stack_ptr = tcbList[0].stack_top - (mainStackAddress - msp);
	tcbs[0].state = running;
	tcbList[0].priority = 0;


	//set msp to main stack base address
	__set_MSP(mainStackAddress);
	//switch msp to psp
	__set_CONTROL(__get_CONTROL() | SPBIT);
	//set PSP to top of new main task (t0)
	__set_PSP(tcbs[0].stack_ptr);
	
}

typedef void(*rtosTaskFunc_t)(void*args);


uint32_t newTask(rtosTaskFunc_t funcPtr, void * args, uint8_t priority) {

	printf("creating new task\n");
	//get empty task 
	int i = 0;

	while (tcbs[i].stack_ptr != tcbs[i].stack_top){
		i++;
		if (i==6) //if no more tasks avail
			return 0;
	}

	//psr default 0x01000000
	tcbs[i].stack_ptr -= 4;
	*((uint32_t *)tcbs[i].stack_ptr) = (uint32_t)0x01000000;

	//pc
	tcbs[i].stack_ptr -= 4;
	*((uint32_t *)tcbs[i].stack_ptr) = (uint32_t)funcPtr;
	
	// LR to R1
	for (int j = 0; j < 5; j++){
		tcbs[i].stack_ptr -= 4;
		*((uint32_t *)tcbs[i].stack_ptr) = (uint32_t)0x0;
	}

	// R0
	tcbs[i].stack_ptr -= 4;
	*((uint32_t *)tcbs[i].stack_ptr) = (uint32_t)args; //DOUBLE CHECK
	

	// R11 to R4
	for (int j = 0; j < 8; j++) {
		tcbs[i].stack_ptr -= 4;
		*((uint32_t *)tcbs[i].stack_ptr) = (uint32_t)0x0;
	}

	//set priority
	tcbs[i].priority = priority;
	tcbs[i].state = ready;

	// add task to queue
	TCB_t * newNode = &tcbs[i];
	enqueue(newNode);

	return 1;

}
uint8_t highestPriority(){
	uint8_t leading0s = __clz(bitVector);

	return (32-leading0s);
}

//context switch
void PendSV_Handler(){

	//find priority of first node in queue
	uint8_t nextPriority =  highestPriority();
	
//	printf( "%d \n", nextPriority);
	
	//check if running task priority is higher than next node
	if (nextPriority>= runningTask->priority) {
		//find next task
		TCB_t * taskNext = dequeue(&queue[nextPriority]);
		
		if (runningTask->state != waiting){
			//add back to queue
			enqueue(runningTask);

			//switch context 

			//push contents of current task onto stack
			runningTask->stack_ptr = (uint32_t*)storeContext();

			// push new task's stack contents to registers
			restoreContext((uint32_t)taskNext->stack_ptr);
			
			if (runningTask->state == running)
				runningTask->state = ready;
			taskNext->state = running;

			runningTask = taskNext;
		}
	
	}
}

void muxInit(sem_t *s, uint32_t count) {
	*(sem_t).semCount = count;
}


void muxAcquire(mux_t *muxCount) {
	__disable_irq();
	
	while(*s <=0){ //loop until count is less than or equal to 0
		__enable_irq();
			if (runningTask->priority > tcbs[(*muxCount).owner].priority) { //check if running task's priority is greater than mutex owner task's priority
		//Priority Inheritance: promote task holding mutex to the highest priority task blocked on the mutex
		tcbs[(*muxCount).owner].priority = runningTask->priority;
		__disable_irq();
	}
	
	(*muxCount).count--; //decrements count to 0 (closed)
	if (*s == 0) {
		muxOwner = runningTask; //if *s equals 
	}
	
	__enable_irq();
}

//Mutex Release Function
void muxRelease(sem_t *s) {
	__disable_irq();
	(*s)++;
	__enable_irq();
}

//Semaphore Initialization
void semInit(sem_t *s, uint32_t count) {
	*(sem_t.semCount) = count;
}

//Semaphore Wait Function
void wait(sem_t *s) {
	__disable_irq();
	(*s)--; //decrements count to 0 (closed)
	if (*s >=0) { 
		__enable_irq();
	}
	else { //count is negative, therefore mutex is not available
		//block task
		runningTask->state = blocked; //change state to blocked
		//add to tail of waitlist
		addToWait(tcbs[i]);
		__enable_irq();
	}
}

//Semaphore Signal Function
void signal(sem_t *s) {
	__disable_irq();
	(*s)++;
	if (*s <=0) { //after incrementing counter if s was negative then we have to unblock the task
		//unblock task
		sem1.waitList->state = running//change state of blocked task to running
		//move from wait list to running list
		enqueue(removeFromWait(tcb[i]));
	}
	__enable_irq();
}


void task1 (void*argument) {
	
	while (1){
		muxAcquire(&lock);
		__disable_irq();
		printf("task 1\n");
		__enable_irq();
		muxRelease(&lock);
	}
	
}

void task2 (void*argument) {
	
	while (1){
		muxAcquire(&lock);
		__disable_irq();
		printf("task 2\n");
		__enable_irq();
		muxRelease(&lock);
	}
	
}

void task3 (void*argument) {
	
	while (1){
		muxAcquire(&lock);
		__disable_irq();
		printf("task 3\n");
		__enable_irq();
		muxRelease(&lock);
	}
}
int main(void) {
	
	//initalize priority queue
	SysTick_Config(SystemCoreClock/1000);
	
	initalizeQueue();
	//Initialize memory map
	initialization();
	
	//Create new tasks
	rtosTaskFunc_t task1ptr = task1;
	rtosTaskFunc_t task2ptr = task2;
	newTask(task1ptr, NULL, 5);
	//newTask(task2ptr, NULL, 1);
//	
//	printf("new task created\n");
//	
//	uint32_t period = 1000; // 1s
//	uint32_t prev = -period;
//	while(true) {
//		if((uint32_t)(msTicks - prev) >= period) {
//			printf("tick ");
//			prev += period;
//		}
//	}
	
	//Initialize Semaphore
	//semInit(&lock, 1); //initialize lock semaphore to 1 (open)

	//Initialize Mutex
	//semInit(&lock, 1); //initialize lock semaphore to 1 (open)
}
