1-First connect the Wi-Fi.
2-Send Hello Drone - Hello Remote. !! The UDP task start ONLY after the wifi init and connection is successful !!
3-The arming of the ESCs should occur only if the pilot decides it from the remote. (For safety).


I have to create a TCP socket. Maybe a connection should be maintained for the critical messages ? 

Confusion between drone_addr and from_addr


I think that I'll create a TCP socket in order to establish a connection. And I'll use this connection for the critical messages : 
1-Hello Drone and Hello Remote.
2-Arming the ESCs.
3-Kill.
4-Every once in a while

I'll use UDP for sending the Setpoints.
