# CORBA AMI

## Introduction

CORBA (Common Object Request Broker Architecture) provides an IDL (Interface Definition Language) 
that allows specifying an interface that is independent of any programming language. 
IDL has a rather simple syntax that has been inspired by C and C++.

CORBA has introduced AMI (Asynchronous Method Invocation) in version 2.4 in October 2000.

Consider the following interface definition:

    module moduleA
    {
        interface interfaceA
        {
            attribute typeA1 attr1;
            readonly attribute typeA2 attr2;
            // more attributes

            typeR operation1(in typeI1 argI1, ..., 
                             inout typeIO1 argIO1, ...,
                             out typeO1 argO1, ...) raises (typeEx1, …);
            void operation2(in typeI1 argI1, ...);
            oneway void operation3(in typeI1 argI1, ...);
            // more operations
        }
    }

Notice that typeA1, typeI1, … are types that should have been defined upfront before using them. 
Their definitions have been omitted in this example.

Although operation2 has only 'in' arguments and a void return type,
it is a blocking operation: the client application can only proceed after the
server has executed the operation and has send a 'void' return value
to the client.

This is not the case for oneway operation3: the client application will proceed immediately
and the server application may execute the operation at a moment the client has advanced
well beyond the operation invocation.
The client may even invoke this oneway operation several times
after each other, queuing up at the server side waiting for execution.

## Synchronous RMI (Remote Method Invocation)

From this IDL, the IDL compiler will generate a stub in the requested programming language 
that will be integrated (compiled and linked) into the client application.
The IDL compiler has generated the following pseudo C++ for operation1:

    namespace moduleA
    {

    typeR interfaceA::operation1(in typeI1 argI1, ..., 
                                 inout typeIO1 argIO1, ...,
                                 out typeO1 argO1, ...)
    {
        Allocate a request buffer.  
        Retrieve information on the destination (IP address, port, target object, ...) 
           from the interfaceA object and write this information to the request.
        Write the identifier for the operation into the request.
        Marshal the in and inout arguments into the request.
        destination.sendRequest(request);     // request buffer will be released by the ORB.
        // -------------------
        reply = destination.waitForReply();   // Calling thread blocks until reply has arrived.
        Inspect the reply for exception information.
        If no exception is present
            Unmarshal the inout and out arguments from the reply.
            Unmarshal the return value from the reply.
            Release the reply buffer.
            return ret;
        Else
            Unmarshal exception information from the reply.
            Release the reply buffer.
            Raise exception;
        End If
    }

    }

The client application can use this operation as follows:
    
    try {
        ret = remoteObject.operation1(argI1, ..., argIO1, ..., argO1, ...);
    }
    catch ( ) {
    
    }

For the server side the IDL compiler generates a callback function (called skeleton) which can look as follows. 

    reply interfaceA_callback(request)
    {
        Retrieve information on the target object from the request and find the target object.
        Retrieve the operation identifier from the request.
        switch (operationId)
        {
            case operationID1:
                Declare variables to contain all argument values.
                Unmarshal in an inout arguments from the request into local variables.
                Call the operation.
                try {
                    // Same signature as the operation on the client side.
                    ret = target.operation1(argI1, ..., argIO1, ..., argO1, ...); 
                }
                catch (Exception1 ) {
                  Allocate reply buffer.
                  Marshal exception into reply.
                  return reply;
                }
                catch (...) {
                  Allocate reply buffer.
                  Marshal exception into reply.
                  return reply;
                }
                Allocate reply buffer.
                Marshal inout an out arguments into reply.
                Marshal return value into reply.
                return reply;

            case operationID2:
                ...
            default:
                // Unknown operation invoked…
        }
    }
	
When the server application registers the target object (called servant) with the ORB, 
the interfaceA_callback function is registered as well. 
The ORB will call this function when a request arrives.
The callback function calls the operation implementation. 
The application developer has to provide this operation implementation.

Apart from the request-reply interaction pattern, CORBA also supports the request-only interaction pattern. 
    
## Asynchronous Method Invocation (AMI)

The IDL compiler can be asked to generate both a synchronous and an asynchronous API.
The IDL compiler generates the following intermediate IDL definition from the one above (I omitted the attributes):

Note that AMI is only applicable on the client side.
There are no changes on the server side.

    module moduleA
    {
        valueType interfaceAPoller;
        interface interfaceAHandler;

        interface interfaceA
        {
            typeR operation1(in typeI1 argI1, ..., 
                             inout typeIO1 argIO1, ...,
                             out typeO1 argO1, ...) raises (typeEx1, …);
            // more operations

            // For every operation we have a sendp_ function that looks as follows:
            interfaceAPoller
                sendp_operation1(in typeI1 argI1, ..., in typeIO1 argIO1, ...);
     
            // For every operation we have a sendc_ function that looks as follows:
            void sendc_operation1(InterfaceAHandler handler,
                                  in typeI1 argI1, ..., in typeIO1 argIO1, ...);
        }
    }

Apart from the synchronous function (operation1), the CORBA IDL compiler also generates 
a sendp_operation1 that starts a polling interaction, and a sendc_operation1 that will result 
in a callback function being invoked on an object the client has created.

The sendp_operation1 and sendc_operation1 functions only use the 'in' and 'inout' arguments of the original operation.

Both mechanisms are described in some more detail now.

### Polling

The sendp_operation1 function returns a value of type interfaceAPoller,
which has the following definition:

    module moduleA
    {
        valuetype interfaceAPoller : Messaging::Poller
        {
            bool operation1(in timeout,
                            out argIO1, ..., 
                            out typeO1 argO1, ...,
                            out typeR ret_val) raises (CORBA::WrongTransaction);
            // more operations
        }
    }

Each operation takes a timeout as first argument, followed by the 'inout' and 'out' arguments of the original operation. 
The last argument corresponds to the return type of the original operation, which is now an 'out' argument. 
The operation returns a Boolean, indicating if the response has arrived. 
If not, the 'out' arguments do not have a meaningful value.

Using timeout value 0, the operation is a non-blocking function: 
it will return immediately after it has checked if the reply has received and filled out the arguments. 
A non-zero value instructs the operation to wait (block) for maximum this value.
A value of -1 means to block until the reply has arrived.

The client application can use polling as follows:

    interfaceAPoller poller;
    try {
        poller = remoteObject.sendp_operation1(argI1, ..., argIO1, ...);
    }
    catch ( ) {
    
    }
    // Do some other work that does not need the reply from the remote object

    // Now poll for the reply:
    try {
        while (!interfaceAPoller.operation1(timeout, argIO1, ..., argO1, ..., retVal) {
            // Sleep for a while or do some other work.
        }
    }
    catch ( ) {
    
    }
    // Remote object has responded: use the results.

It is easy to use this interface to start a polling operation on several remote objects, 
do some other work, and then poll for all the replies.

The application may use an array of remote objects, an array of poller objects, 
and a counter to count how many objects have replied.
    
### Callback

In the case of callback, the IDL compiler generates an additional interface:

    module moduleA
    {
        interface interfaceAHandler
        {
            void operation1(in typeIO1 argIO1, ..., 
                            in typeO1 argO1, ..., 
                            in typeR ret_val);
            void operation1_exception(typeEx1);

            // more operations
        }
    }

The application developer has to provide an implementation for the operations in this interface, 
which is usually done in a class derived from interfaceAHandler, e.g. interfaceAHandler_impl. 

The following IDL definition illustrates this, although this interface will not be generated in IDL format,
but has to be implemented directly in C++.

    module moduleA
    {
        interface interfaceAHandler_impl : interfaceAHandler
        {
            void operation1(in typeIO1 argIO1, ..., 
                            in typeO1 argO1, ..., 
                            in typeR ret_val);
            bool operation1ReplyReceived();
            void operation1_exception(typeEx1);

            // more operations
        }
    }
	
The developer can/should add one or more member variables (and access functions) 
so that interfaceAHandler_impl::operation1 can save the result for later use by the application.
Here we have added operation1ReplyReceived().
    
The application creates an object interfaceAHandler_impl and will pass this object as first argument 
to the sendc_operation1. 
The ORB will save this object and invoke a callback function when the remote object has responded. 

The client application can use this operation as follows:

    interfaceAHandler_impl* hndlr = new interfaceAHandler_impl();
    try {
        ret = remoteObject.sendc_operation1(hndlr, argI1, ..., argIO1, ...);
    }
    catch ( ) {
    
    }

    // Do some other work that does not need the reply from the remote object

    // Now enter a local event loop:
    do {
        run_event_loop(timeout);
    } while (!hndlr.operation1ReplyReceived());
    // Reply of operation1 has arrived.
    // Read the content of interfaceAHandler_impl hndlr.

The client application will now also act as server: it will have to enter an event loop 
(when this is appropriate in the application’s control flow), 
so that the ORB can callback the operations in the interface handler.
The application programmer has added operation1ReplyReceived() 
to interfaceAHandler_impl that returns true if the ORB performed the callback (operation1).

If the application was originally a pure client, then using the callback mechanism 
may have an impact on the architecture of the application. 

Again, it is easy to use this interface to start a callback operation on several remote objects, 
do some other work, and then enter the event loop.

The application may use an array of remote objects, 
an array of callback objects, and a counter to count how many objects have replied.

C# uses another approach in which the callback function will be invoked on a separate thread.
The main thread will have to check if the callback function has already been executed.
One way is to use a variable that is set by the callback function and checked 'polled' by the main thread.
Another way is to use a semaphore: the main application can wait on a semaphore that is set in the callback function.

This section only gave a very high-level introduction to AMI. 
The reader is referred to [Schmidt et al.] for more information on CORBA AMI.

## C++ coroutines

Using C++ coroutines (in corolib), we can rewrite the example as follows:

	async_task<int> coroutine1 {
	
        struct ret_type {
            typeIO1 argIO1; 
            ...;
            typeO1 argO1;
            ...:
            typeR retV;
            typeEx1 ex1;
            ...
        };
        
        async_operation<ret_type> op = remoteObject.send_operation1(argI1, ..., argIO1, ...);

        // Do some other work that does not need the reply from the remote object
        
        ret_type ret_val;
        ret_val = co_await op; // The coroutine will return control to its calling function/coroutine 
                               // if the reply has not been received.
		// The coroutine will continue from this point when the reply has been received.
		...
		
	    co_return 0;
	}
	
First we define define a struct that will contain all in/out and out arguments, 
a variable to contain the return value
and variables that contains the exceptions.
This type is then passed as a template type to async_operation.

Because of the use of co_await, this code will have to be placed in a coroutine, i.e. a function
that returns a value of type async_task and that uses co_return to return from the coroutine.

An advantage of coroutines (compared to CORBA AMI) is that the coroutine will not block at co_await until the reply has been received,
but that control is returned to the caller or resumer of the coroutine, so that other coroutines can proceed.

A disadvantage of coroutines is that more manual coding is necessary, 
unless we could generate coroutine code from an IDL definition.

## C++ example (CORBA)

To conclude this brief introduction to CORBA AMI, the following example uses an IDL defining 
an operation that takes one argument (of command1_t) and returns a value of response1_t 
and it shows the C++ classes generated from it.

    module moduleA
    {
        valuetype command1_t;
        valuetype response1_t;

        interface interfaceA
        {
            response1_t operation1(in command1_t command);
        };
    };

The IDL compiler generates the following C++ code (some details have been removed):

    class moduleA
    {
    public:
        class interfaceA : virtual public CORBA::Object {
        public:
            moduleA::response1_t operation1(const moduleA::command1_t& command);

            void sendc_operation1(moduleA::interfaceA_Handler_ptr ami_handler,
                                  const moduleA::command1_t& command);

            interfaceA_Poller sendp_operation1(const moduleA::command1_t& command);
        };

        class interfaceA_Poller : public Messaging::Poller {
        public:
            CORBA::Boolean operation1(CORBA::ULong timeout, moduleA::response1_t& ami_return_val);
        };

        class interfaceA_Handler : virtual public POA_Messaging::ReplyHandler {
        public:
            virtual void operation1(const moduleA::response1_t& ami_return_val)= 0;
            virtual void operation1_excep(CORBA::Environment *ev)= 0;
       };
    };

    class interfaceA_Handler_impl : virtual public moduleA::interfaceA_Handler {
    public: 
        virtual void operation1(const moduleA::response1_t&  ami_return_val);
        void operation1_excep( CORBA::Environment *ev);
    };

## References

[Schmidt et al.] Object Interconnections – Programming Asynchronous Method Invocations with CORBA Messaging (Column 16), Douglas C. Schmidt and Steve Vinoski,
https://www.dre.vanderbilt.edu/~schmidt/PDF/C++-report-col16.pdf
