package main;

use strict;
use warnings;

my %ThoLink_sets = (
	"raw"               => "",
	"GroupId"           => "g",
	"AckMode"           => "a",
	"RssiThresholdDB"   => "t",
	"LED"               => "l",
	"Reset"             => "r"
);

#--------------------------------------------------------------------------------------------------
sub ThoLink_Initialize($)
{
  my ($hash) = @_;
  Log3 $hash, 5, "ThoLink_Initialize";

  require "$attr{global}{modpath}/FHEM/DevIo.pm";

  $hash->{DefFn}		= "ThoLink_Define";
  $hash->{UndefFn}		= "ThoLink_Undef";

  $hash->{AttrList}     = "$readingFnAttributes";
  #$hash->{GetFn}        = "ThoLink_Get";
  $hash->{SetFn}        = "ThoLink_Set";

  $hash->{ReadyFn}		= "ThoLink_Ready";
  $hash->{ReadFn}		= "ThoLink_Read";
  $hash->{WriteFn}		= "ThoLink_Write";
}

#--------------------------------------------------------------------------------------------------
sub ThoLink_Define($$)
{
  my ($hash, $def) = @_;
  Log3 undef,1, "ThoLink_Define def=".$def;
  Log3 $hash, 5, "ThoLink_Define";

  DevIo_CloseDev($hash);
  
  my @a = split("[ \t][ \t]*", $def);

  my $name = $a[0];
  my $dev = $a[2];

  $dev .= "\@57600" if($dev !~ m/\@/);
  $hash->{DeviceName} = $dev;

  $hash->{Clients} = ":TN";
  my %mc = (
	"1:TN" => "^OK "
  );
  $hash->{MatchList} = \%mc;
  
  my $ret = DevIo_OpenDev($hash, 0, undef);
  return $ret;
}

#--------------------------------------------------------------------------------------------------
sub ThoLink_Undef($$)
{
  my ($hash, $arg) = @_;
  Log3 $hash, 5, "ThoLink_Undef";

  DevIo_CloseDev($hash);
  return undef;
}

#--------------------------------------------------------------------------------------------------
sub ThoLink_Ready($)
{
	my ($hash) = @_;
	
	return DevIo_OpenDev($hash, 1, undef )
	  if ( $hash->{STATE} eq "disconnected" );

	# This is relevant for windows/USB only
	my $po = $hash->{USBDev};
	my ( $BlockingFlags, $InBytes, $OutBytes, $ErrorFlags ) = $po->status;
	return ( $InBytes > 0 );
}

#--------------------------------------------------------------------------------------------------
sub ThoLink_Write($$)
{
	my ($hash, $msg) = @_;
	my $name = $hash->{NAME};

	Log3 $hash, 0, "$name/ThoLink: WRITE: $msg";
	
	DevIo_SimpleWrite($hash, $msg, 0);
}

#--------------------------------------------------------------------------------------------------
sub ThoLink_Read($)
{
  my ($hash) = @_;
  my $name = $hash->{NAME};

  my $buf = DevIo_SimpleRead($hash);		
  if ( !defined($buf) )
  {
  	Log3 $hash, 1, "$name/ThoLink EMPTY-READ. RETRY ...";
	$buf = DevIo_SimpleRead($hash);		
	if ( !defined($buf) )
	{
		Log3 $hash, 1, "$name/ThoLink DOUBLE-EMPTY-READ. REOPEN ...";
		DevIo_CloseDev($hash);
		DevIo_OpenDev($hash, 1, undef);
		return "";
	}
	else
	{
		Log3 $hash, 4, "$name/ThoLink READ-RETRY: $buf";
	}
  }
  else
  {
    Log3 $hash, 4, "$name/ThoLink READ: $buf"; 
  }

  my $pandata = $hash->{PARTIAL};
  Log3 $hash, 5, "$name/ThoLink/RAW: $pandata/$buf";
  $pandata .= $buf;

  while($pandata =~ m/\n/) 
  {
    my $rawmsg;
    ($rawmsg,$pandata) = split("\n", $pandata, 2);
    $rawmsg =~ s/\r//;
    
	Log3 $hash, 4, "$name/ThoLink/LINE: $rawmsg";
	
	if($rawmsg =~ m/^\[ThoGateway\:\:([^\]]*)\]/)
	{
		Log3 $hash, 0, "$name/ThoLink: Device: ThoGateway=$1";
		
		readingsSingleUpdate($hash, "ThoGateway", $1, 1);
	}
	elsif($rawmsg =~ m/^(\w+):\s+([^\s]+)/)
	{
		my $readingName = $1;
		my $readingValue = $2;
		Log3 $hash, 0, "$name/ThoLink: DeviceSetting: $readingName=$readingValue";
		
		readingsSingleUpdate($hash, $readingName, $readingValue, 1);
	}
	elsif($rawmsg =~ m/^OK\s/)
	{
		my %addvals = (RAWMSG => $rawmsg);
		Dispatch($hash, $rawmsg, \%addvals)
	}
	else
	{
		Log3 $hash, 0, "$name/ThoLink: Unrecognized message: $rawmsg";
	}
  }
  $hash->{PARTIAL} = $pandata;
}

#--------------------------------------------------------------------------------------------------
sub ThoLink_Set($@)
{
	my ( $hash, @a ) = @_;
	return "\"set X\" needs at least an argument" if ( @a < 2 );
	
	my $name = shift @a;
	my $opt = shift @a;
	my $value = join("", @a);
	
	if(!defined($ThoLink_sets{$opt})) 
	{
		my @cList = keys %ThoLink_sets;
		return "Unknown argument $opt, choose one of " . join(" ", @cList);
	}
	
	Log3 $hash, 0, "$name/ThoLink: SET: $opt=$value";
	
	if($opt eq "raw")
	{
		ThoLink_Write($hash, "$value");
	}
	elsif($opt eq "GroupId")
	{
		ThoLink_Write($hash, "$value"."g");
	}
	elsif($opt eq "AckMode")
	{
		ThoLink_Write($hash, (($value eq "off")?"0":"1")."a");
	}
	elsif($opt eq "RssiThresholdDB")
	{
		ThoLink_Write($hash, -$value."t");
	}
	elsif($opt eq "LED")
	{
		ThoLink_Write($hash, (($value eq "off")?"0":"1")."l");
	}
	elsif($opt eq "Reset")
	{
		ThoLink_Write($hash, "r");
	}
	else
	{
		Log3 $hash, 0, "$name/ThoLink: SET: unknown command $opt=$value";
	}
}

#--------------------------------------------------------------------------------------------------
1;