/* 
* Player.h
*
* Created: 11/06/2021 09:45:18
* Author: philg
*/


#ifndef __PLAYER_H__
#define __PLAYER_H__


class CPlayer
{
//variables
public:
    bool m_IsPlaying;
protected:
private:
    uint32_t m_NoteDuration_us;
    uint32_t m_NoteStart_us;
    const char* m_SequenceStr;
    uint8_t m_Tempo;
    uint8_t m_DefaultNoteLength;
    uint8_t m_Volume;
    size_t m_Next;
    uint8_t m_Octave;
    float m_SilenceDuration;
    bool m_Repeat;
    enum NodeMode_t 
	{
	    MODE_NORMAL,
	    MODE_LEGATO,
	    MODE_STACCATO
    } m_eNoteMode;

	
//functions
public:
	CPlayer();
	~CPlayer();
	void Update();
	// initialise ready to play string
	void PrepareToPlayString(const char* SequenceStr);
	void Play(const char* string);
	void Stop();
	void StartSilence(float duration);
	void StartNote(float duration, float frequency, float volume);
	char NextChar();
	uint8_t NextNumber();
	size_t NextDots();
	float RestDuration(uint32_t rest_length, uint8_t dots);

	// Called when the MML player should start the next action
	void NextAction();

protected:
private:
	CPlayer( const CPlayer &c );
	CPlayer& operator=( const CPlayer &c );

}; //Player

#endif //__PLAYER_H__
