/* 
* Player.cpp
*
* Created: 11/06/2021 09:45:18
* Author: philg
*/

#include "Includes.h"
#include "ctype.h"
#include "CPlayer.h"

uint32_t PlayTime;

// default constructor
CPlayer::CPlayer()
{
} //Player

// default destructor
CPlayer::~CPlayer()
{
} //~Player



void CPlayer::Update()
{
	PlayTime = Core.micros()-m_NoteStart_us;
	// Check if note is over
	if (m_IsPlaying && Core.micros()-m_NoteStart_us > m_NoteDuration_us) 
	{
		NextAction();
	}
}


void CPlayer::Play(const char* string)
{
	PrepareToPlayString(string);
	NextAction();
}



void CPlayer::PrepareToPlayString(const char* SequenceStr)
{
	Stop();
	m_SequenceStr				= SequenceStr;
	m_Next							= 0;
	m_Tempo						= 120;
	m_DefaultNoteLength	= 4;
	m_eNoteMode				= MODE_NORMAL;
	m_Octave						= 4;
	m_Volume						= 255;
	m_SilenceDuration			= 0;
	m_Repeat						= false;
	m_NoteDuration_us = 0;
	m_IsPlaying = true;
}


void CPlayer::Stop()
{
	m_IsPlaying = false;
	Tone.noTone(5);
}

void CPlayer::StartSilence(float duration)
{
	m_NoteStart_us = Core.micros();
	m_NoteDuration_us = duration*1e6;
	Tone.noTone(5);
}

void CPlayer::StartNote(float duration, float frequency, float volume)
{
	m_NoteStart_us = Core.micros();
	m_NoteDuration_us = duration*1e6;
	Tone.PlayTone(PORTA,5,frequency ,m_NoteDuration_us/1000U);
}


char CPlayer::NextChar()
{
	while (m_SequenceStr[m_Next] != '\0' && isspace(m_SequenceStr[m_Next])) 
	{
		m_Next++;
	}

	return toupper(m_SequenceStr[m_Next]);
}

uint8_t CPlayer::NextNumber()
{
	uint8_t ret = 0;
	while (isdigit(NextChar())) 
	{
		ret = (ret*10) + (NextChar() - '0');
		m_Next++;
	}
	return ret;
}

size_t CPlayer::NextDots()
{
	size_t ret = 0;
	while (NextChar() == '.') 
	{
		ret++;
		m_Next++;
	}
	return ret;
}

float CPlayer::RestDuration(uint32_t rest_length, uint8_t dots)
{
	float whole_note_period = 240.0f / m_Tempo;
	if (rest_length == 0) 
	{
		rest_length = 1;
	}

	float rest_period = whole_note_period/rest_length;
	float dot_extension = rest_period/2;

	while (dots--) 
	{
		rest_period += dot_extension;
		dot_extension *= 0.5f;
	}

	return rest_period;
}

void CPlayer::NextAction()
{
	if (m_SilenceDuration > 0) 
	{
		StartSilence(m_SilenceDuration);
		m_SilenceDuration = 0;
		return;
	}

	uint8_t note = 0;
	uint8_t note_length;

	while (note == 0) 
	{
		char c = NextChar();
		if (c == '\0') 
		{
			if (m_Repeat) 
			{
				// don't "play" here, as we may have been called from there, and it turns out infinite recursion on
				// invalid strings is suboptimal.  The next call to update() will push things out as appropriate.
				PrepareToPlayString(m_SequenceStr);
			} 
			else 
			{
				Stop();
			}
			return;
		}

		m_Next++;
		switch (c) 
		{
		case 'V': 
				m_Volume = NextNumber();
				break;
		case 'L': 
				m_DefaultNoteLength = NextNumber();
				if (m_DefaultNoteLength == 0) 
				{
					Stop();
					return;
				}
				break;
		case 'O':
			m_Octave = NextNumber();
			if (m_Octave > 6) 
				m_Octave = 6;
			break;
		case '<':
			if (m_Octave > 0) 
				m_Octave--;
			break;
		case '>':
			if (m_Octave < 6) 
				m_Octave++;
			break;
		case 'M':
			c = NextChar();
			if (c == '\0') 
			{
				Stop();
				return;
			}
			m_Next++;
			switch (c) 
			{
			case 'N':
				m_eNoteMode = MODE_NORMAL;
				break;
			case 'L':
				m_eNoteMode = MODE_LEGATO;
				break;
			case 'S':
				m_eNoteMode = MODE_STACCATO;
				break;
			case 'F':
				m_Repeat = false;
				break;
			case 'B':
				m_Repeat = true;
				break;
			default:
				Stop();
				return;
			}
			break;
		case 'R':
		case 'P': 
			{
				uint8_t num = NextNumber();
				uint8_t dots = NextDots();
				StartSilence(RestDuration(num, dots));
				return;
			}
		case 'T':
			m_Tempo = NextNumber();
			if (m_Tempo < 32) 
			{
				Stop();
				return;
			}
			break;
		case 'N':
			note = NextNumber();
			note_length = m_DefaultNoteLength;
			if (note > 84) 
			{
				Stop();
				return;
			}
			if (note == 0) 
			{
				uint8_t num = NextNumber();
				uint8_t dots = NextDots();
				StartSilence(RestDuration(num, dots));
				return;
			}
			break;
		case 'A':
		case 'B':
		case 'C':
		case 'D':
		case 'E':
		case 'F':
		case 'G': 
			{
				static const uint8_t note_tab[] = {9,11,0,2,4,5,7};
				note = note_tab[c-'A'] + (m_Octave*12) + 1;

				c = NextChar();

				switch (c) 
				{
				case '#':
				case '+':
					if (note < 84) 
						note++;
					m_Next++;
					break;
				case '-':
					if (note > 1) 
						note--;
					m_Next++;
					break;
				default:
					break;
				}
				note_length = NextNumber();
				if (note_length == 0) 
					note_length = m_DefaultNoteLength;
				break;
			}
			default:
			Stop();
			return;
		}
	}

	// Avoid division by zero
	if (m_Tempo == 0 || note_length == 0) 
	{
		Stop();
		return;
	}

	float note_period = 240.0f / (float)m_Tempo / (float)note_length;

	switch (m_eNoteMode) 
	{
	case MODE_NORMAL:
		m_SilenceDuration = note_period/8;
		break;
	case MODE_STACCATO:
		m_SilenceDuration = note_period/4;
		break;
	case MODE_LEGATO:
		m_SilenceDuration = 0;
		break;
	}
	note_period -= m_SilenceDuration;

	float dot_extension = note_period/2;
	uint8_t dots = NextDots();
	while (dots--) 
	{
		note_period += dot_extension;
		dot_extension *= 0.5f;
	}
	CMyMath Math;
	float note_frequency = 880.0f * expf(logf(2.0f) * ((int)note - 46) / 12.0f);
	float note_volume = m_Volume/255.0f;
	note_volume *= 100 * 0.01;
	note_volume = Math.constrain_float(note_volume, 0, 1);

	note_frequency = Math.constrain_float(note_frequency, 10, 22000);

	StartNote(note_period, note_frequency, note_volume);
}
