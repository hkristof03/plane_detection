#include "control.h"


void mouse_callback(int event, int x, int y, int flags, void* userdata)
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
			if (x < m_x) u -= 1.0;
			else if (x > m_x) u += 1.0;
			if (y < m_y) v -= 1.0;
			else if (y > m_y) v += 1.0;

			m_x = x;
			m_y = y;

			event_cnt++;
		}
	}
	else if (event == cv::EVENT_LBUTTONUP)
	{
		m_x = 0;
		m_y = 0;
	}
	else if (event == cv::EVENT_MOUSEWHEEL)
	{
		if (cv::getMouseWheelDelta(flags) > 0)
		{
			rad /= (float)1.1;
			event_cnt++;
		}

		else if (cv::getMouseWheelDelta(flags) < 0)
		{
			rad *= (float)1.1;
			event_cnt++;
		}
	}
}