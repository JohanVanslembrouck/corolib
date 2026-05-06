# Using Boost ASIO with corolib

## Contents

* Folder common contains the source code to build a library that is used by the client-server applications in the following folders.

* Folders clientserver0, clientserver1, clientserver2, clientserver3 and clientserver4 use the common library for building their applications.

* Folder commonlso (lso stands for lazy start operations) contains the source code to build a library that is used by the applications in folder clientserver1lso.

* Folder clientserver1x uses the common and commonlso libraries for building its applications.

* Folder various contains stand-alone applications. At this moment it contains only timer applications.
