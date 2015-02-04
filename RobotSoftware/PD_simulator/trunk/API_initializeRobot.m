function [result]=API_initializeRobot()
%Calibration
%SelfTest
%OpenRobot
clear all;
close all;
dirpath=pwd;
open_system([dirpath,'/SABiR_simulator']);
result=0;