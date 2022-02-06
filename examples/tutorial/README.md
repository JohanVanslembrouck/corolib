# Tutorial

This directory contains various introductory examples to the corolib library.

The tutorial does not depend on the Boost library or on Qt.

Some remarks:
* The file name of the examples reflect the used classes. However, because all examples use async_task to create coroutines, async_task does not appear in the file name.
* Most examples use classes defined in corolib. Some use classes defined locally in this tutorial.
* The coroutines defined in the example files are not defined as member functions of a class, but at the global scope. In a way they are "C-style" coroutines.

The following gives a brief description of all examples in this tutorial.

* p1000.cpp is the simplest example. 
  It only uses async_task. Even the "leaf function" is a coroutine (coroutine5). 
  The coroutines coroutine1 till coroutine4 will not be suspended, so they will not have to be resumed: 
  the co_await calls will resume immediately. The same behavior is possible with functions only.

* p1002-no-coroutine.cpp implements the same behavior as p1000.cpp without coroutines
  (i.e. with traditional functions).
  It demonstrates that the use of coroutines in p1000.exe did not have any added value.

* p1010-thread.cpp is a clumsy way to introduce suspend-and-resume behavior to p1000.cpp.
  It uses the implementation details of coroutines (promise_type, initial_suspend, get_return_object, 
  final_suspend) which is certainly not a good idea.

* p1050-resume_same_thread.cpp introduces a struct resume_same_thread that implements 
  await_ready, await_suspend and await_resume. The call to await_ready returns true,
  which makes that await_suspend is not called. Instead await_resume is called immediately after
  await_ready. The coroutines in this example are not suspended.

* p1052-resume_same_thread.cpp is a variant of p1050-resume_same_thread.cpp.
  The call to await_ready returns false, which makes that await_suspend is called afterwards.
  The function await_suspend resumes the coroutines: await_resume is called from inside await_suspend.
  The coroutines in this example are not suspended.

* p1060-resume_new_thread.cpp is based upon p1052-resume_same_thread.cpp, but await_suspend resumes the 
  coroutine from a thread after a delay of 1 second. 
  The coroutines in this example are suspended and will be resumed from the thread created in await_suspend.
  
* p1100-auto_reset_event.cpp uses the auto_reset_event class from corolib. An object 'are1' of this type is
  created at the global level. Coroutine coroutine2 co_await-s this object that is resumed from main.

* p1110-auto_reset_event-one_way_tasks.cpp uses a oneway_task and 4 auto_reset_event objects.
  It is not possible to co_await the completion of a oneway_task: once a oneway_task is started, it runs
  until completion.
  However, using auto_reset_event objects, it is possible to wait for the completion of a oneway_task 
  in an indirect way.
  It is also possible to "advance" a oneway_task that co_await-s one or more auto_reset_event objects 
  in its body.

* p1120-auto_reset_event-thread.cpp resumes an auto_reset_event from a thread created by the leaf coroutine,
  coroutine5,
  instead of from the main function as in p1100-auto_reset_event.cpp.

* p1200-mini0.cpp uses mini0 that is defined in the tutorial directory.
  The class mini0 can be considered to be a simplication of async_operation<void>.
  A the same time, mini0 is a small extension of the class resume_same_thread
  defined in p1050-resume_same_thread.cpp and p1052-resume_same_thread.cpp.
  The example creates a mini0 object at the file level.
  The main() function calls the resume() function on this object twice.

* p1210-mini1-thread.cpp uses mini1 that is also defined in the tutorial directory.
  The class mini1 can be considered to be a simplification of async_operation<T>.
  The example creates a mini1 object in the leaf coroutine, coroutine5.
  A thread function will resume the mini1 object after a delay of 1 second.

* p1220-mini1-oneway_task-thread.cpp is a variant of p1210-mini1-thread.cpp where
  coroutine3 does not return an async_task<int> object as usual, but a oneway_task object.
  Also couroutine4 is more complicated. It implements a for-loop that can (will) be
  canceled from coroutine3 after 10 seconds.

* p1300-future.cpp

The following examples come in triples 
(p14X0-async_operation.cpp, p14X2-async_operation-eventqueue.cpp, p14X4-async_operation-thread.cpp)
and implement the same flow but in a different way. The result should be same for each triple.
These examples are the core of this tutorial because they use both async_task and async_operation 
as in the examples using the Boost library and Qt.

* The p14X0-async_operation.cpp examples resume the coroutines in the main() function in a manual way.
  This means that the programmer has to know the coroutines to be completed and the order in which they have to be completed.

* The p14X2-async_operation-eventqueue.cpp resumes the coroutines in an automated way.
  This is accomplished using an event queue. Every coroutine that has to be resumed registers 
  a lambda function in the event queue.
  The run() function iterates over the queue and calls the call operator on the lambda function.
  The event queue is rather simple: only lambda functions of a single type (std::function<void(int)>)
  can be registered. The run() function always passes 10 are argument.

* The p14X4-async_operation-thread.cpp examples do not use an event queue, but call the lambda functions 
  from a thread after a delay of one second.
  The main() function calls get_result() on the first coroutine (coroutine1).
  This will block the main() function until completion of all coroutines.

The following describes implementation of the examples per triple.

* p140X.cpp

* p141X.cpp

* p142X.cpp

* p143X.cpp

* p144X.cpp

* p145X.cpp

* p146X.cpp
