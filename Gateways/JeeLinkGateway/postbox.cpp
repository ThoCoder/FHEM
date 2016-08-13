#include "postbox.h"

// ----------------------------------------------------------------------------------------------------
Postbox::Postbox()
{
	ClearAllEntries();
}

// ----------------------------------------------------------------------------------------------------
bool Postbox::SetEntry(byte nodeId, byte* data, byte datalen)
{
	if (datalen == 0)
		return false;
	if (datalen > POSTBOXMAXMSGLEN)
		return false;
	if (data == NULL)
		return false;

	PostboxEntry* entry = GetEntry(nodeId);
	if (entry == NULL)
		entry = GetFreeEntry();
	if (entry == NULL)
		return false;

	memcpy(entry->Data, data, datalen);
	entry->DataLen = datalen;
	entry->NodeId = nodeId;

	//Serial.print("Postbox: new entry: ");
	//DumpEntry(entry);
	return true;
}

// ----------------------------------------------------------------------------------------------------
PostboxEntry* Postbox::GetEntry(byte nodeId)
{
	for (byte i = 0; i < POSTBOXSIZE; i++)
	{
		PostboxEntry* entry = &m_Entries[i];

		if (entry->NodeId == nodeId)
			return entry;
	}

	return NULL;
}

// ----------------------------------------------------------------------------------------------------
PostboxEntry* Postbox::GetFreeEntry()
{
	for (byte i = 0; i < POSTBOXSIZE; i++)
	{
		PostboxEntry* entry = &m_Entries[i];

		if (entry->NodeId == 255)
			return entry;
	}

	return NULL;
}

// ----------------------------------------------------------------------------------------------------
void Postbox::ClearEntry(PostboxEntry* entry)
{
	//Serial.print("Postbox: clearing entry: ");
	//DumpEntry(entry);

	entry->NodeId = 255;
	entry->DataLen = 0;
	return;
}

// ----------------------------------------------------------------------------------------------------
void Postbox::ClearEntry(byte nodeId)
{
	for (byte i = 0; i < POSTBOXSIZE; i++)
	{
		PostboxEntry* entry = &m_Entries[i];

		if (entry->NodeId == nodeId)
		{
			ClearEntry(entry);
			return;
		}
	}
}

// ----------------------------------------------------------------------------------------------------
void Postbox::ClearAllEntries()
{
	for (byte i = 0; i < POSTBOXSIZE; i++)
	{
		PostboxEntry* entry = &m_Entries[i];

		entry->NodeId = 255;
		entry->DataLen = 0;
	}

	//Serial.println("Postbox: All entries cleared.");
}

// ----------------------------------------------------------------------------------------------------
void Postbox::DumpEntry(PostboxEntry* entry)
{
	Serial.print("ID:");
	Serial.print(entry->NodeId, DEC);
	Serial.print(" LEN:");
	Serial.print(entry->DataLen, DEC);
	Serial.print(" DATA:");

	for (byte d = 0; d < entry->DataLen; d++)
	{
		Serial.print(" ");
		Serial.print(entry->Data[d], DEC);
	}

	Serial.println();
}

// ----------------------------------------------------------------------------------------------------
void Postbox::Dump()
{
	Serial.println("Postbox:");

	for (byte i = 0; i < POSTBOXSIZE; i++)
	{
		PostboxEntry* entry = &m_Entries[i];

		if (entry->NodeId != 255)
		{
			DumpEntry(entry);
		}
	}
}

// ----------------------------------------------------------------------------------------------------

