
#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include "DIALOG.h"

extern void fanControl(_Bool value);
extern void _UserDraw(WM_HWIN hWin, int Stage);
WM_HWIN CreateMainWindow(void);

PROGBAR_Handle _hProgbar;       // Handle of progressbar
GRAPH_DATA_Handle  _ahDataI;  	// Handle of GRAPH_DATA I
GRAPH_DATA_Handle  _ahDataU;  	// Handle of GRAPH_DATA U
GRAPH_SCALE_Handle _hScaleV;    // Handle of vertical scale
GRAPH_SCALE_Handle _hScaleH;    // Handle of horizontal scale
SLIDER_Handle _hSlider;         // Handle of slider
GRAPH_Handle _hGraph;			// Handle of graph
TEXT_Handle _hTextVoltage;		// Handle of Text for Voltage

#endif /* MAINWINDOW_H_ */
