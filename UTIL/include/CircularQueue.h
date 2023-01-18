/*
 * CircularQueue.h
 *
 *  Created on: 2022. 4. 15.
 *      Author: hc.ro
 */

#ifndef CIRCULARQUEUE_H_
#define CIRCULARQUEUE_H_

#define TRUE 1
#define FALSE 0

typedef struct rxData
{
    char mode;
    char data[9];
} rxDataPc;

#define QUE_LEN 50                // Packet byte (data length)

typedef rxDataPc Data;

typedef struct _cQueue
{
    int front;
    int rear;
    Data queArr[QUE_LEN];
    int state;                      // Queue state Empty or full
} CQueue;

typedef CQueue Queue;

void QueueInit(Queue *pq);
int QIsEmpty(Queue *pq);

void Enqueue(Queue *pq, Data data);
Data Dequeue(Queue *pq);
Data QPeek(Queue *pq);

#endif /* CIRCULARQUEUE_H_ */
