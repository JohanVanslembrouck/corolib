# Tutorial

This directory contains various introductory examples to the corolib library.

The tutorial does not depend on the Boost library or on Qt.

Some remarks:
* The file name of the examples reflect the used classes.
  However, because all examples use async_task to create coroutines, 
  async_task will not appear in the file name.
* Most examples use classes defined in corolib.
  Some use classes defined locally in this tutorial.
* The coroutines in the examples are not defined as member functions of a class,
  but at the global scope. In a way they are "C-style" coroutines.

The following gives a brief description of all examples in this tutorial.

* p1000.cpp is the simplest example. 
  It only uses async_task. Even the "leaf function" is a coroutine (coroutine5). 
  The coroutines coroutine1 till coroutine4 will not be suspended, so they will not have to be resumed: 
  the co_await calls will resume immediately. The same behavior is possible with functions only.

* p1002-no-coroutine.cpp implements the same behavior as p1000.cpp without coroutines
  (i.e. with traditional functions).
  It demonstrates that the use of coroutines in p1000.exe did not have any added value.

* p1010-thread.cpp is a clumsy way to introduce suspend-and-resume behavior to p1000.cpp.
  It uses implementation details of coroutines (promise_type, initial_suspend, get_return_object, 
  final_suspend) which is certainly not a good idea.

* p1050-resume_same_thread.cpp introduces a struct resume_same_thread that implements 
  await_ready, await_suspend and await_resume. The call to await_ready returns true;
  consequently await_suspend is not called.
  Instead await_resume is called immediately after await_ready.
  The coroutines in this example are not suspended.

* p1052-resume_same_thread.cpp is a variant of p1050-resume_same_thread.cpp.
  The call to await_ready returns false, which makes that await_suspend is called afterwards.
  The function await_suspend resumes the coroutines: await_resume is called from inside await_suspend.
  The coroutines in this example are not suspended.

* p1054-resume_same_thread.cpp has the same behavior as p1052-resume_same_thread.cpp.
  struct resume_same_thread defines operator co_await and an awaiter type that implements 
  await_ready, await_suspend and await_resume.
  This example uses more lines of code to have the same behavior.
  For larger classes, it is more useful to group these functions
  in an awaiter type to separate them from functions added to the main class resume_same_thread.

* p1060-resume_new_thread.cpp is based upon p1052-resume_same_thread.cpp, but await_suspend resumes the 
  coroutine from a thread after a delay of 1 second. 
  The coroutines in this example are suspended and will be resumed from the thread created in await_suspend.
  
* p1064-resume_new_thread.cpp has the same behavior as p1060-resume_new_thread.cpp.
  struct resume_new_thread defines operator co_await and an awaiter type that implements 
  await_ready, await_suspend and await_resume.
  This example is the equivalent of p1054-resume_same_thread.cpp.

* p1100-auto_reset_event.cpp uses the auto_reset_event class from corolib.
  An object 'are1' of this type is created at the global level.
  Coroutine coroutine2 co_await-s this object that is resumed from main.

* p1110-auto_reset_event-one_way_tasks.cpp uses a oneway_task and 4 auto_reset_event objects.
  It is not possible to co_await the completion of a oneway_task: once a oneway_task is started, it runs
  until completion.
  However, using auto_reset_event objects, it is possible to wait for the completion of a oneway_task 
  in an indirect way.
  It is also possible to "advance" a oneway_task that co_await-s one or more auto_reset_event objects 
  in its body.

* p1120-auto_reset_event-thread.cpp resumes an auto_reset_event from a thread created by the leaf coroutine,
  coroutine5, instead of from the main function as in p1100-auto_reset_event.cpp.

* p1200-mini0.cpp uses mini0 that is defined in the tutorial directory.
  The class mini0 can be considered to be a simplification of async_operation<void>.
  A the same time, mini0 is a small extension of the class resume_same_thread
  defined in p1054-resume_same_thread.cpp.
  The example creates a mini0 object at the file level.
  The main() function calls the resume() function on this object twice.

* p1210-mini1-thread.cpp uses mini1 that is also defined in the tutorial directory.
  The template class mini1 can be considered to be a simplification of async_operation<T>.
  The example creates a mini1 object in the leaf coroutine, coroutine5.
  A thread function will resume the mini1 object after a delay of 1 second.

* p1220-mini1-oneway_task-thread.cpp is a variant of p1210-mini1-thread.cpp.
  coroutine3 does not return an async_task<int> object as usual, but a oneway_task object.
  Also coroutine4 is somewhat more complicated.
  It implements a for-loop that can (will) be canceled from coroutine3 after 10 seconds.

* p1300-future.cpp uses the future type that Microsoft has extended with coroutine functionality
  and that comes with the installation of Microsoft Studio 2019.
  This implementation spawns additional threads,
  possibly because this was the only way to avoid modifications
  to the original implementation of future/promise.

    Note that this example used to work with the experimental implementation of coroutines. It does not work anymore with the final implementation.

The following examples come in groups
(p14X0-async_operation.cpp, p14X2-async_operation-eventqueue.cpp, p14X4-async_operation-thread.cpp, 
p14X6-async_operation-immediate.cpp, p14X8-async_operation-evtq-imm.cpp, p14X9-async_operation-thread-imm.cpp)
Each triple implements the same flow in a different way.
These examples are the core of this tutorial because they use both async_task and async_operation 
as in the examples using the Boost library and Qt.

* The p14X0-async_operation.cpp examples resume the coroutines in the main() function in a manual way.
  This means that the programmer has to know the coroutines to be completed
  and the order in which they have to be completed.

* The p14X2-async_operation-eventqueue.cpp resumes the coroutines in an automated way.
  This is accomplished using an event queue.
  Every coroutine that has to be resumed registers a lambda function in the event queue.
  The run() function iterates over the queue and calls the call operator on the lambda function.
  The event queue is rather simple: only lambda functions of a single type (std::function<void(int)>)
  can be registered.
  The run() function always passes 10 are argument.

* The p14X4-async_operation-thread.cpp examples do not use an event queue, but call the lambda functions 
  from a thread after a delay of one second.
  The main() function calls get_result() on the first coroutine (coroutine1).
  This will block the main() function until completion of all coroutines.

* The p14X6-async_operation-immediate.cpp examples start asychronous operations that complete immediately.

* The p14X8-async_operation-evtq-imm.cpp examples start one asynchronous operation that is placed on an event queue for later completeion and a second
  that completes immediately.

* The p14X9-async_operation-thread-imm.cpp examples start one asynchronous operation that will be completed from a thread and a second
  that completes immediately.
  
The following describes implementation of the examples per group.

* p140X.cpp defines a function start_op that creates a lambda object that will be assigned to a global variable.
  The lambda object contains a reference to an async_operation object.
  When the call operator is called on the lambda object,
  the coroutine that co_await-s the async_operation object will be resumed.

* p141X.cpp uses Class01 that has a start_op member function.
  Class01 inherits from CommService.
  This example is close to the style used in the clientserverX directories.
  These examples use one instance of Class01.

* p142X.cpp use two instances of Class01.
  An operation is started asynchronously on each object of this class.
  The completion of both operations is awaited by when_all.

* p143X.cpp define two leaf coroutines, coroutine5a and coroutine5b, that are started from coroutine4.
  The completion of both coroutines is awaited by when_all.

* p144X.cpp is based on p143X.cpp.
  The two leaf coroutines, coroutine5a and coroutine5b,
  implement a loop that invoke the operations twice.

* p145X.cpp is based on p144X.cpp.
  In p145X.cpp, coroutine3 calls coroutine4 twice instead of once as in p144X.cpp

* p146X.cpp defines two objects of class Class02 that defines 2 operations.
  It is based on p145X.cpp, but the two leaf coroutines, coroutine5a and coroutine5b,
  invokes both operations on each of the objects.

* p147X.cpp is based on p146X.cpp.
  coroutine5a and coroutine5b now implement a while loop,
  in a similar way as a thread function is often implemented.
  coroutine5a and coroutine5b remain in the loop as long as variable running remains true.
  Their sibling coroutine5c will set this variable to false after 30 seconds.
  coroutine4 awaits the completion of the three coroutine5 variants.
  There is no p1470-async_operation.cpp, because the manual resumption from main is difficult 
  in this example and will be different for other timeouts used in coroutine5c.

* p150X.cpp contains a coroutine that, when it is resumed, has to complete a coroutine that it has under its control.

* p160X.cpp demonstrates "split-and-combine". See p1600.cpp for further explanation.

* p170X.cpp demonstrates the use of async_ltask.

* p180X.cpp contains a single coroutine1 that co_awaits in a loop the completion of global async_operation<int> op.
  This object is completed several times. coroutine1 co_returns when it receives the value 0.
