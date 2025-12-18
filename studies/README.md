# corolib studies

The following subdirectories do not use any third-party communication frameworks,
apart from some examples in corolab that use Boost ASIO.

* [why-coroutines](./why-coroutines) contains examples that explain the advantages of C++ coroutines for writing (distributed) asynchronous applications.

* [why-coroutines2](./why-coroutines2) contains examples that explain the advantages of C++ coroutines for writing asynchronous I/O-based applications.

* [corolab](./corolab) contains examples that do not use corolib, but were the basis from where corolib has been "distilled".

* [corba](./corba) studies the various CORBA invocation variants (synchronous, asynchronous using callback, 
asynchronous using (non-)blocking polling) and adds the use of coroutines.

* [initial_suspend](./initial_suspend) studies lazy- and eager-start coroutines, i.e. coroutines that return std::suspend_always 
or std::suspend_never at their initial suspend point, respectively.

* [final_suspend](./final_suspend) studies the use of various types returned at the final suspend point,
observing which of them assure deleting all coroutine frames to avoid memory leaks.

* [rvo](./rvo) studies the use of return value optimization (RVO) in the definition of async_operation (used in corolib),
and the impact the absence/presence of RVO has on the correctness of the results.

* [transform](./transform) contains various examples that illustrate how a C++ compiler may transform coroutine code 
to C++ code that can be compiled with a C++ compiler that does not support coroutines.

See the README.md file in these directories for further information.
