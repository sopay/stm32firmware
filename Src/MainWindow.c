#include "MainWindow.h"

#define ID_WINDOW_0 (GUI_ID_USER + 0x0A)
#define ID_SLIDER_0 (GUI_ID_USER + 0x0F)
#define ID_CHECKBOX_0 (GUI_ID_USER + 0x10)
#define ID_CHECKBOX_1 (GUI_ID_USER + 0x11)
#define ID_CHECKBOX_2 (GUI_ID_USER + 0x12)
#define ID_TEXT_0 (GUI_ID_USER + 0x13)
#define ID_GRAPH_0 (GUI_ID_USER + 0x15)
#define ID_PROGBAR_0 (GUI_ID_USER + 0x16)
#define ID_TEXT_1 (GUI_ID_USER + 0x17)
#define ID_BUTTON_0 (GUI_ID_USER + 0x18)
#define ID_TEXT_2 (GUI_ID_USER + 0x19)

static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 0, 0, 480, 272, 0, 0x0, 0 },
  { SLIDER_CreateIndirect, "Slider", ID_SLIDER_0, 156, 40, 308, 20, 0, 0x0, 0 },
  { CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_0, 28, 14, 98, 20, 0, 0x0, 0 },
  { CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_1, 28, 42, 93, 20, 0, 0x0, 0 },
  { CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_2, 28, 70, 91, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Gate Voltage:", ID_TEXT_0, 159, 15, 85, 17, 0, 0x0, 0 },
  { GRAPH_CreateIndirect, "Graph", ID_GRAPH_0, 156, 65, 308, 160, 0, 0x0, 0 },
  { PROGBAR_CreateIndirect, "Progbar", ID_PROGBAR_0, 11, 127, 134, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Heatsink Temperature", ID_TEXT_1, 14, 108, 132, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "0%", ID_TEXT_2, 230, 15, 85, 17, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Start", ID_BUTTON_0, 120, 230, 233, 35, 0, 0x0, 0 },
};

static _Bool _Start = 0;

extern void StartMeasuring(void);
extern void StopMeasuring(void);

void setSliderValue(int value);
void _UserDraw(WM_HWIN hWin, int Stage);

/*********************************************************************
*
*       setSliderValue
*
* Function description
*   Maps data from slider to text widget
*/
void setSliderValue(int value)
{

  char str[15];
  value = SLIDER_GetValue(_hSlider);
  sprintf(str, "%d%%", value);
  TEXT_SetText(_hTextVoltage, str);

}

/*********************************************************************
*
*       _UserDraw
*
* Function description
*   This routine is called by the GRAPH object before anything is drawn
*   and after the last drawing operation.
*/
void _UserDraw(WM_HWIN hWin, int Stage)
{

  if (Stage == GRAPH_DRAW_LAST) {

    char acText[] = "Volt";
    GUI_RECT Rect;
    GUI_RECT RectInvalid;
    int FontSizeY;

    GUI_SetFont(&GUI_Font13_ASCII);
    FontSizeY = GUI_GetFontSizeY();
    WM_GetInsideRect(&Rect);
    WM_GetInvalidRect(hWin, &RectInvalid);
    Rect.x1 = Rect.x0 + FontSizeY;
    GUI_SetColor(GUI_YELLOW);
    GUI_DispStringInRectEx(acText, &Rect, GUI_TA_HCENTER, strlen(acText), GUI_ROTATE_CCW);

  }

}

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg)
{

  int     NCode;
  int     Id;

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:

    _hButton = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
    _hProgbar = WM_GetDialogItem(pMsg->hWin, ID_PROGBAR_0);
    _hSlider = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_0);
    _hGraph = WM_GetDialogItem(pMsg->hWin, ID_GRAPH_0);
    _hTextVoltage = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
    _hCheckboxA = WM_GetDialogItem(pMsg->hWin, ID_CHECKBOX_0);
    _hCheckboxB = WM_GetDialogItem(pMsg->hWin, ID_CHECKBOX_1);
    _hCheckboxC = WM_GetDialogItem(pMsg->hWin, ID_CHECKBOX_2);
    _ahDataI = GRAPH_DATA_YT_Create(GUI_RED, 500, 0, 0);
    _ahDataU = GRAPH_DATA_YT_Create(GUI_MAGENTA, 500, 0, 0);
    _hScaleV = GRAPH_SCALE_Create(20, GUI_TA_RIGHT, GRAPH_SCALE_CF_VERTICAL, 40);
    //_hScaleH = GRAPH_SCALE_Create(5, GUI_TA_HCENTER, GRAPH_SCALE_CF_HORIZONTAL, 40);

    CHECKBOX_SetText(_hCheckboxA, "MOSFET A/B");
    CHECKBOX_SetText(_hCheckboxB, "MOSFET C/D");
    CHECKBOX_SetText(_hCheckboxC, "MOSFET E/F");

//    GRAPH_SetGridDistY(_hGraph, 10);
//    GRAPH_SetGridVis(_hGraph, 1);
//    GRAPH_SetGridFixedX(_hGraph, 1);
//    GRAPH_SetUserDraw(_hGraph, _UserDraw);

    GRAPH_SCALE_SetTextColor(_hScaleV, GUI_YELLOW);
    GRAPH_AttachScale(_hGraph, _hScaleV);

    //GRAPH_SCALE_SetTextColor(_hScaleH, GUI_YELLOW);
    //GRAPH_AttachScale(_hGraph, _hScaleH);

    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_SLIDER_0: // Notifications sent by 'Slider'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // Optionally insert code for reacting on notification message
        break;
      case WM_NOTIFICATION_RELEASED:
        // Optionally insert code for reacting on notification message
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
    	  setSliderValue(SLIDER_GetValue(_hSlider));
        // Optionally insert code for reacting on notification message
        break;
      // Optionally insert additional code for further notification handling
      }
      break;
    case ID_CHECKBOX_0: // Notifications sent by 'Checkbox'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // Optionally insert code for reacting on notification message
        break;
      case WM_NOTIFICATION_RELEASED:
        // Optionally insert code for reacting on notification message
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // Optionally insert code for reacting on notification message
        break;
      // Optionally insert additional code for further notification handling
      }
      break;
    case ID_CHECKBOX_1: // Notifications sent by 'Checkbox'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // Optionally insert code for reacting on notification message
        break;
      case WM_NOTIFICATION_RELEASED:
        // Optionally insert code for reacting on notification message
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // Optionally insert code for reacting on notification message
        break;
      // Optionally insert additional code for further notification handling
      }
      break;
    case ID_CHECKBOX_2: // Notifications sent by 'Checkbox'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // Optionally insert code for reacting on notification message
        break;
      case WM_NOTIFICATION_RELEASED:
        // Optionally insert code for reacting on notification message
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // Optionally insert code for reacting on notification message
        break;
      // Optionally insert additional code for further notification handling
      }
      break;
    case ID_BUTTON_0: // Notifications sent by 'Start'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // Optionally insert code for reacting on notification message
    	  if (_Start) {
    		  _Start = 0;
    		  BUTTON_SetText(_hButton, "Start");
    		  StopMeasuring();
    	  } else {
    		  _Start = 1;
    		  BUTTON_SetText(_hButton, "Stop");
    		  StartMeasuring();
    	  }

        break;
      case WM_NOTIFICATION_RELEASED:
        // Optionally insert code for reacting on notification message
        break;
        // Optionally insert additional code for further notification handling
      }
      break;
    // Optionally insert additional code for further Ids
    }
    break;
  // Optionally insert additional message handling
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       CreateWindow
*/
WM_HWIN CreateMainWindow(void);
WM_HWIN CreateMainWindow(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

