#include "control.h"

void MouseCallBackFunc(int event, int x, int y, int flags, void* userdata)
{

	if (event == cv::EVENT_LBUTTONDOWN)
	{
		m_x = x;
		m_y = y;
	}
	else if (event == cv::EVENT_MOUSEMOVE)
	{
		if (m_x != 0 && m_y != 0)
		{
			if (x < m_x) u -= 0.1;
			else if (x > m_x) u += 0.1;
			if (y < m_y) v -= 0.1;
			else if (y > m_y) v += 0.1;

			m_x = x;
			m_y = y;
		}
	}
	else if (event == cv:EVENT_LBUTTONUP)
	{
		m_x = 0;
		m_y = 0;
	}
	else if (event == cv::EVENT_MOUSEWHEEL)
	{
		if (cv::getMouseWheelDelta(flags) > 0) rad /= (float)1.1;

		else if (cv::getMouseWheelDelta(flags) < 0) rad *= (float)1.1;
	}
	drawPoints(mrw, u, v, rad);
}