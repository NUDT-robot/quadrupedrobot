/*
 * Author: Chang, Xu
 *
int main()
{
    Gnuplot gp;
    int index = gp.CreatePlotFile("/home/cx/gp.txt");
    cout << "Index is " << index << endl;
    if(index == Gnuplot::NoSpace)
        return 1;
    string str;
    str.append("123456\t888\n");
    gp.RecordData(index, str);
    gp.ClosePlotFile(index);

}

*/
#include <iostream>
#include <fstream>

class Gnuplot
{
	// Max number of files simultaneously supported.
#define MAXNFILENUMBER 128
	// Store output stream pointer;
    std::ofstream* osArray[MAXNFILENUMBER];
	// 1: Distributed; 0: Idle.
	bool osArrayStatus[MAXNFILENUMBER];
	// Flush frequency and counter
	uint flushfrequency = 1000;
	uint osArrayWriteCounter[MAXNFILENUMBER];
public:
	// ERROR CODE
	enum ERROR
	{
		Success = 0,
		IndexOutOfRange = -11,
		NotAllocated = -12,
		NoSpace = -13
	};
private:
	// Dispense a file index number in range from 0 to MAXNFILENUMBER-1
	// It will dispense a idle position with a minimum index number, return EORROR::NoSpace if no idle position.
	int DispenseFileNumber();
	bool ValidIndex(int index);
public:
	Gnuplot();
    ~Gnuplot();
	bool IsDistributed(int index);
	inline void SetFlushFrequency(uint freq){ flushfrequency = std::max(freq,(uint)1);}
    int CreatePlotFile(const std::string& filename);
    int RecordData(int index, const std::string& str);
	int ClosePlotFile(int index);
};
