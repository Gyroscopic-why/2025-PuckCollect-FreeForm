#pragma once

template <typename T>
class Queue {
private:
    struct Node {
        T data;
        Node* next;
        Node(const T value) : data(value), next(nullptr) {}
    };

    Node* head;
    Node* tail;

public:
    Queue() : head(nullptr), tail(nullptr) {}

    ~Queue() {
        while (!isEmpty()) {
            dequeue();
        }
    }

    void enqueue(const T value) {
        Node* newNode = new Node(value);
        
        if (isEmpty()) {
            head = tail = newNode;
        } else {
            tail->next = newNode;
            tail = newNode;
        }
    }

    void dequeue() {
        if (isEmpty()) return;

        Node* temp = head;
        head = head->next;
        
        if (head == nullptr) {
            tail = nullptr;
        }
        
        delete temp;
    }
    
    T front() {
        if (isEmpty()) {
            static T dummy;
            return dummy;
        }
        return head->data;
    }
    
    T frontAndDequeue(){
        T data = front();
        
        dequeue();
        
        return data;
    }

    bool isEmpty() const {
        return head == nullptr;
    }
};