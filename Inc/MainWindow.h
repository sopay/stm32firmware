#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include <stdio.h>
#include <string.h>
#include "DIALOG.h"

PROGBAR_Handle _hProgbar;     // Handle of progressbar
GRAPH_DATA_Handle _ahDataI;  	// Handle of GRAPH_DATA I
GRAPH_DATA_Handle _ahDataU;  	// Handle of GRAPH_DATA U
GRAPH_SCALE_Handle _hScaleV;  // Handle of vertical scale
GRAPH_SCALE_Handle _hScaleH;  // Handle of horizontal scale
SLIDER_Handle _hSlider;       // Handle of slider
GRAPH_Handle _hGraph;		      // Handle of graph
TEXT_Handle _hTextVoltage;	  // Handle of Text for Voltage
BUTTON_Handle _hButton;       // Handle of startbutton
CHECKBOX_Handle _hCheckboxA;  // Handle of checkbox for module A/B
CHECKBOX_Handle _hCheckboxB;  // Handle of checkbox for module C/D
CHECKBOX_Handle _hCheckboxC;  // Handle of checkbox for module E/F

#endif /* MAINWINDOW_H_ */
