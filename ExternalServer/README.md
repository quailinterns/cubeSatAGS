{\rtf1\ansi\ansicpg1252\cocoartf1265\cocoasubrtf200
{\fonttbl\f0\fswiss\fcharset0 Helvetica;}
{\colortbl;\red255\green255\blue255;}
\margl1440\margr1440\vieww10800\viewh8400\viewkind0
\pard\tx720\tx1440\tx2160\tx2880\tx3600\tx4320\tx5040\tx5760\tx6480\tx7200\tx7920\tx8640\pardirnatural

\f0\fs24 \cf0 This will help you get a server running with the cube satellite data. It uses a serial-port, most likely from an xbee, and pushes that data using node.js to a server using an http request. This currently uses heroku, a free way to make a server with 250 web hours. \
\
1. 	Go to heroku and install the heroku desktop terminal command.\
2.	Login as:\
		username: quailinterns@gmail.com\
		password: quail123\
3.	Create a new heroku app.\
4.	Login using the terminal by typing the command \'93heroku login\'94. \
5.	Push all of the files in the folder \'93herokuFiles\'94 to your created server using the terminal command lines. The terminal command lines are very similar to pushing to github. Everything you need to get up and running is in the folder.\
6.	Open up the postdata.py python script and change the serial port if needed and server url to the one that you created. The default was the url and serial port we used during the demonstration. \
7.	Save and run the python script on the computer that has the xbee connected. Navigate to your website server and the data should all appear on the web, accessible by anyone on the web. \
\
As a side note, the python script has a boolean for a local or nonlocal server. Using the command \'93foreman start\'94 in terminal when logged into heroku will host the server on localhost:5000. }