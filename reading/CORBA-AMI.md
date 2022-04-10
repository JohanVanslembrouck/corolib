# CORBA AMI

## Introduction

CORBA (Common Object Request Broker Architecture) has introduced AMI (Asynchronous Method Invocation) in version 2.4 in October 2000.
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
			oneway void operation2(in typeI1 argI1, ...);
			// more operations
		}
	}

The IDL compiler can be asked to generate both a synchronous and an asynchronous API.
In a programming language that has the same syntax as IDL, the generated interface looks as follows:

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

			// For every operation we have a sendp_ function looking as follows:
			interfaceAPoller
				sendp_operation1(in typeI1 argI1, ..., in typeIO1 argIO1, ...);
	 
			// For every operation we have a sendc_ function looking as follows:
			void sendc_operation1(InterfaceAHandler handler,
								  in typeI1 argI1, ..., in typeIO1 argIO1, ...);
		}
	}

Apart from the synchronous function (operation1), the CORBA IDL compiler also generates 
a sendp_operation1 that starts a polling interaction, and a sendc_operation1 that will result 
in a callback function being invoked on an object the client has created.
The sendp_ and sendc_  functions only use the 'in' and 'inout' arguments of the original operation.

Both mechanisms are described in some more detail now.

## Polling

The sendp_ function returns a valueof type interfaceAPoller, which has the following definition:

	module moduleA
	{
		valuetype interfaceAPoller
		{
			bool get_operation1(in timeout,
								out argIO1, ..., out typeO1 argO1, ...,
								out typeR ret_val);
			// more operations
		}
	}

For every operation in the IDL, the poller has an operation whose name is prefixed with get_. 
The operation takes a timeout as first argument, followed by the 'out' arguments of the original operation. 
The last argument corresponds to the return type of the original operation, which is now an 'out' argument. 
The get_ function returns a Boolean, indicating if the response has arrived. 
If not, the 'out' arguments do not have a meaningful value. 
Using timeout value 0, the get_ operation is a non-blocking function: 
it will return immediately after it has checked if the reply has received and filled out the arguments. 
A non-zero value instructs the get_ function to wait (block) for maximum this value.

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
        while (!interfaceAPoller(timeout, argIO1, ..., argO1, ..., retVal) {
            // Sleep for a while or do some other work.
        }
    }
    catch ( ) {
    
    }
    // Remote object has responded: use the results.

It is easy to use this interface to start a polling operation on several remote objects in a loop, 
do some other work, and then use another loop polling for the replies. 
The application may use an array of remote objects, an array of poller objects, 
and a counter to count how many objects have replied.
    
## Callback

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

The application developer has to provide an implementation of operations in this interface, 
which is usually done in a class derived from interfaceAHandler, e.g. interfaceAHandler_impl. 
The developer can/should add one or more member variables (and access functions) 
so that interfaceAHandler_impl::operation1 can save the result for later use by the application.

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
	
The application creates an object interfaceAHandler_impl and will pass this object as first argument 
to the sendc_operation1. 
The ORB will save this object and invoke a callback function when the remote object has responded. 

The client application can use this operation as follows:

    interfaceAHandler_impl hndlr = new interfaceAHandler_impl();
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
to interfaceAHandler_impl that returns true if the ORB performed the callback.

If the application was originally a pure client, then using the callback mechanism 
may have an impact on the architecture of the application. 
If the application was designed as a server application then it already had an event loop 
and the callback functions can/will be called from this event loop.

Again, it is easy to use this interface to start a callback operation on several remote objects in a loop, 
do some other work, and then enter the event loop. The application may use an array of remote objects, 
an array of callback objects, and a counter to count how many objects have replied.

This section only gave a very high-level introduction to AMI. 
The reader is referred to [Schmidt et al.] for more information on CORBA AMI.

## C++ example

To conclude this brief introduction to CORBA AMI, the following example uses an IDL defining 
an operation that takes one argument (of command1_t) and returns a value of response1_t. 

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

		class interfaceA_Poller {
		public:
			CORBA::Boolean operation1Poller(int timeout, moduleA::response1_t& ami_return_val);
		};

		class interfaceA_Handler : virtual public CORBA::Object {
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

> 	[Schmidt et al.] Object Interconnections – Programming Asynchronous Method Invocations with CORBA Messaging (Column 16)
	Douglas C. Schmidt and Steve Vinoski
	https://www.dre.vanderbilt.edu/~schmidt/PDF/C++-report-col16.pdf
