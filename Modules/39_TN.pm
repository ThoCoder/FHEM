package main;

use strict;
use warnings;

#--------------------------------------------------------------------------------------------------
sub TN_Initialize($)
{
  my ($hash) = @_;
  Log3 $hash, 5, "TN_Initialize";

  # Match/Prefix
  my $match = "TN";
  $hash->{Match}     = "^TN";
  $hash->{DefFn}     = "TN_Define";
  $hash->{UndefFn}   = "TN_Undef";
  $hash->{ParseFn}   = "TN_Parse";
  $hash->{AttrList}  = "$readingFnAttributes";
  
  # TNCnf
  # $data{TNCnf}{<SensorType>}{ReadingName}
  # $data{TNCnf}{<SensorType>}{DataBytes}
  # $data{TNCnf}{<SensorType>}{Scale}
  # $data{TNCnf}{<SensorType>}{Offset}
  # $data{TNCnf}{<SensorType>}{Function}
  
  # Temperature -------------------------------------------
  $data{TNCnf}{11}{ReadingName} = "temperature";
  $data{TNCnf}{11}{DataBytes} = 2;
  $data{TNCnf}{11}{Scale} = 0.1;
  # Temperature (LaCrosse)-------------------------------------------
  $data{TNCnf}{12}{ReadingName} = "temperature";
  $data{TNCnf}{12}{DataBytes} = 2;
  $data{TNCnf}{12}{Scale} = 0.1;
  $data{TNCnf}{12}{Offset} = -40;
  # Humidity ----------------------------------------------
  $data{TNCnf}{16}{ReadingName} = "humidity";
  $data{TNCnf}{16}{DataBytes} = 1;
  # ThoLink RSSI ------------------------------------------
  $data{TNCnf}{100}{ReadingName} = "RSSI";
  $data{TNCnf}{100}{DataBytes} = 1;
  # Gassensor ---------------------------------------------
  $data{TNCnf}{193}{ReadingName} = "Hall";
  $data{TNCnf}{193}{DataBytes} = 2;
  $data{TNCnf}{194}{ReadingName} = "Signal";
  $data{TNCnf}{194}{DataBytes} = 1;
  $data{TNCnf}{195}{ReadingName} = "Count";
  $data{TNCnf}{195}{DataBytes} = 4;
  # Klingel -----------------------------------------------
  $data{TNCnf}{192}{ReadingName} = "RingRequestId";
  $data{TNCnf}{192}{DataBytes} = 1;
  $data{TNCnf}{196}{ReadingName} = "BellSenderRSSI";
  $data{TNCnf}{196}{DataBytes} = 1;
  $data{TNCnf}{197}{ReadingName} = "BellConnectionOk";
  $data{TNCnf}{197}{DataBytes} = 1;
  $data{TNCnf}{198}{ReadingName} = "BellRinging";
  $data{TNCnf}{198}{DataBytes} = 1;
  $data{TNCnf}{199}{ReadingName} = "BellSrcNodeId";
  $data{TNCnf}{199}{DataBytes} = 1;
  # AVR Voltage -------------------------------------------
  $data{TNCnf}{252}{ReadingName} = "PowerSupply";
  $data{TNCnf}{252}{DataBytes} = 2;
  $data{TNCnf}{252}{Scale} = 0.0001;
}

#--------------------------------------------------------------------------------------------------
sub TN_Define($$)
{
  my ($hash, $def) = @_;
  Log3 $hash, 5, "TN_Define";
	
  my @a = split("[ \t]+", $def);

  if(@a != 5) 
  {
    my $msg = "wrong syntax: define <name> TN <nodeType> <nodeGroup> <nodeId>";
    Log3 $hash, 0, $msg;
    return $msg;
  }

  my $name = $a[0];
  my $nodeType = $a[2];
  my $nodeGroup = $a[3];
  my $nodeId = $a[4];
  my $nodeCode = ($nodeType << 16) + ($nodeGroup << 8) + $nodeId;

  if(defined($modules{TN}{defptr}{$nodeCode})) 
  {
    return "Node $nodeType $nodeGroup $nodeId ($nodeCode) already defined";
  }
  
  $hash->{CODE} = $nodeCode;
  $hash->{STATE} = "NEW: " . TimeNow();
  $hash->{OrderID} = $nodeCode;

  $modules{TN}{defptr}{$nodeCode} = $hash;
  return undef;
}

#--------------------------------------------------------------------------------------------------
sub TN_Undef($$)
{
  my ($hash, $arg) = @_;
  Log3 $hash, 5, "TN_Undef";

  my $nodeCode = $hash->{CODE};

  if(defined($modules{TN}{defptr}{$nodeCode}))
  {
    delete $modules{TN}{defptr}{$nodeCode};
  }
 
  return undef;
}

#--------------------------------------------------------------------------------------------------
sub TN_Parse($$) 
{
  my ($iodev, $rawmsg) = @_;

  # $rawmsg = TN nodeType nodeGroup nodeId rawData[]

  Log3 $iodev, 5, "TN_Parse: " . $rawmsg . " IODEV:" . $iodev->{NAME};
  
  my @data = split(/\s+/, $rawmsg);
  my $nodeType = $data[1];
  my $nodeGroup = $data[2];
  my $nodeId = $data[3];
  my $nodeCode = ($nodeType << 16) + ($nodeGroup << 8) + $nodeId;
  my @rawData = @data[4..$#data];

  Log3 $iodev, 4, "TN_Parse: T:$nodeType/G:$nodeGroup/N:$nodeId ($nodeCode): dataLen=" . @rawData;
  
  my ($hash, $name);
  if(defined($modules{TN}{defptr}{$nodeCode})) 
  {
	$hash = $modules{TN}{defptr}{$nodeCode};
	$name = $hash->{NAME};
  }
  else
  {
    Log3 $iodev, 1, "TN_Parse: T:$nodeType/G:$nodeGroup/N:$nodeId ($nodeCode): UNDEFINED. rawData = " . join(' ', @rawData);
	return "UNDEFINED TN:$nodeCode";
  }

  my %readings;
  while(@rawData > 0)
  {
	my $sensorType = shift(@rawData);
	
	if($sensorType =~ /\(-(\d+)\)/)
	{
		my $rssi = $1;
		$readings{"RSSI"} = $rssi;

		Log3 $hash, 5, "TN_Parse: $name/T:$nodeType/G:$nodeGroup/N:$nodeId ($nodeCode): RSSI = $rssi";
	}
	elsif(defined($data{TNCnf}{$sensorType}{ReadingName}))
	{
		my $readingName = $data{TNCnf}{$sensorType}{ReadingName};
		my $dataSize = $data{TNCnf}{$sensorType}{DataBytes};
		
		if($dataSize > @rawData)
		{
			Log3 $hash, 1, "TN_Parse: $name/T:$nodeType/G:$nodeGroup/N:$nodeId ($nodeCode): MISSING DATA BYTES FOR SENSOR TYPE $sensorType";
		}
		else
		{
			my $rawValue = 0;
			for(my $i=0; $i<$dataSize; $i++)
			{
				$rawValue += (int(shift(@rawData)) << ($i * 8));
			}

			my $scale = 1;
			if(defined($data{TNCnf}{$sensorType}{Scale}))
			{
				$scale = $data{TNCnf}{$sensorType}{Scale};
			}
			my $offset = 0;
			if(defined($data{TNCnf}{$sensorType}{Offset}))
			{
				$offset = $data{TNCnf}{$sensorType}{Offset};
			}
				
			my $sensorValue = $rawValue*$scale + $offset;
			
			$readings{$readingName} = $sensorValue;

			Log3 $hash, 5, "TN_Parse: $name/T:$nodeType/G:$nodeGroup/N:$nodeId ($nodeCode): $readingName($sensorType) = $sensorValue ($rawValue * $scale + $offset)";
		}
	}
	else
	{
		Log3 $hash, 1, "TN_Parse: $name/T:$nodeType/G:$nodeGroup/N:$nodeId ($nodeCode) UNKNOWN SENSOR TYPE $sensorType";
	}
  }
  
  readingsBeginUpdate($defs{$name});
  foreach my $r (sort keys %readings) 
  {
	readingsBulkUpdate($defs{$name}, $r, $readings{$r});
  }
  readingsEndUpdate($defs{$name}, 1);
  
  return $name;
}

#--------------------------------------------------------------------------------------------------
1;
