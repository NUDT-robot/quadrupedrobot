#include "include/gnuplot.hh"
#include <cmath>

using namespace std;

Gnuplot::Gnuplot()
{
	for(int i = 0; i < MAXNFILENUMBER; i++)
	{
		osArrayWriteCounter[i] = 1;
	}
}
Gnuplot::~Gnuplot()
{
    for (int i = 0; i < MAXNFILENUMBER; i++)
    {
        if(osArrayStatus[i] == true)
        {
            ClosePlotFile(i);
        }
    }
}
bool Gnuplot::ValidIndex(int index)
{
	if(index < 0 or index > MAXNFILENUMBER - 1)
	{
		cout << "Gnuplot::RecordData, index is out of range !" << endl;
		return false;
	}
	else
		return true;
}

int Gnuplot::DispenseFileNumber()
{
	for(int i = 0; i < MAXNFILENUMBER; i++)
	{
		if(osArrayStatus[i] == false)
		{
			return i;
		}
	}
	return NoSpace;
}

bool Gnuplot::IsDistributed(int index)
{
	if(!ValidIndex(index))
		return false;
	return osArrayStatus[index];
}

int Gnuplot::CreatePlotFile(const string& filename)
{
	// Dispense a number
	int index = DispenseFileNumber();
	if(index == NoSpace)
	{
		cout << "Create file failed, because no idle file position exists." << endl;
		return index;
	}
	// Save a new ofstream instance
	osArray[index] = new ofstream;
	osArray[index]->open(filename);
	if(!osArray[index]->is_open())
	{
		cout << "Error opening file " << filename << endl;
	}
	osArrayStatus[index] = true;
	return index;
}

int Gnuplot::RecordData(int index, const string& str)
{
	if(!ValidIndex(index))
		return IndexOutOfRange;
	if(!osArrayStatus[index])
	{
		cout << "Index " << index << "is not allocated." << endl;
		return NotAllocated;
	}
	*osArray[index] << str;
	if(osArrayWriteCounter[index] % flushfrequency == 0)
	{
		osArray[index]->flush();
	}
	return Success;
}

int Gnuplot::ClosePlotFile(int index)
{
	if(!ValidIndex(index))
		return IndexOutOfRange;
	if(!osArrayStatus[index])
	{
		cout << "Index " << index << "is not allocated." << endl;
		return NotAllocated;
	}
	osArray[index]->flush();
	osArray[index]->close();
	delete osArray[index];
	osArrayStatus[index] = false;
	return Success;
}
