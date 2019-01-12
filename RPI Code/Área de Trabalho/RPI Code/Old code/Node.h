#ifndef NODE_H
#define	NODE_H

#include <string>
#include <iostream>

using namespace std;
template <typename T>
class Node {
private:
  T data;
  Node<T>* next;
public:
  Node(T);
  virtual ~Node(); 

  template <typename U> friend class List;
};
template <typename T>
Node<T>::Node(T d) {
  data = d;
  next = NULL;
}
template <typename T>
Node<T>::~Node() {
}
#endif	/* STRNODE_H */
