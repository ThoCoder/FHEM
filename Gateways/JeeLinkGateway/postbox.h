#include "arduino.h"

#define POSTBOXSIZE 16
#define POSTBOXMAXMSGLEN 14

struct PostboxEntry
{
	byte NodeId;
	byte DataLen;
	byte Data[POSTBOXMAXMSGLEN];
};

class Postbox
{
	PostboxEntry m_Entries[POSTBOXSIZE];

public:
	Postbox();

public:
	bool SetEntry(byte nodeId, byte* data, byte datalen);
	PostboxEntry* GetEntry(byte nodeId);
	PostboxEntry* GetFreeEntry();
	void ClearEntry(PostboxEntry* entry);
	void ClearEntry(byte nodeId);
	void ClearAllEntries();
	void DumpEntry(PostboxEntry* entry);
	void Dump();
};

