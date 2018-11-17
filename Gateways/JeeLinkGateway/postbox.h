#include "arduino.h"

#define POSTBOXSIZE 30
#define POSTBOXMAXMSGLEN 29

struct PostboxEntry
{
    byte GroupId;
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
	bool SetEntry(byte groupId, byte nodeId, byte* data, byte datalen);
	PostboxEntry* GetEntry(byte groupId, byte nodeId);
	PostboxEntry* GetFreeEntry();
	void ClearEntry(PostboxEntry* entry);
	void ClearEntry(byte groupId, byte nodeId);
	void ClearAllEntries();
	void DumpEntry(PostboxEntry* entry);
	void Dump();
};

