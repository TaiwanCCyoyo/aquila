
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <map>
#include <cstdint>

#include "verilated_vcd_c.h"
#include "Vaquila_testharness.h"
#include "Vaquila_testharness__Syms.h"
#include "verilated.h"

#include "sim_mem.h"

#define TRACE
//#undef TRACE
#define FENCE_ENABLE
#undef FENCE_ENABLE
#define MAX_SIM_CYCLE 1000000

using namespace std;
static vluint64_t cpuTime = 0;
uint32_t tohost_addr = 0;
double sc_time_stamp() { return cpuTime; }
Vaquila_testharness* top;
VerilatedVcdC* Vcdfp;

void load_simple_asm();

static void usage(const char * program_name)
{
  cout << "Usage: " << program_name << " [RISCV_TEST_ELF] [RVTEST(0/1),default 0]" <<endl;
}


int main(int argc, char **argv)
{
  Verilated::commandArgs(argc,argv);
  map<string,uint64_t> elf_symbols;
  bool rv_test_enable = false;

  if (argc < 2) {
    usage(argv[0]);
    cerr << "Please provide riscv test elf file" << endl;
    return -1;
  }

  fstream log_file("cpu.log",fstream::out);

  if (!log_file.is_open()) {
    cerr << "Failed to open cpu.log file!!!" << endl;
    return -1;
  }

  

  if(argc < 3 || argv[3][0] == '0'){
    cout << "Execute code in TCM\n";
    fstream bin_file, mem_file;
    bin_file.open(argv[1],ios::in | ios::binary);
    mem_file.open("/home/twccyoyo/riscv/aquila_verilate/aquila/ip_repo/aquila/hdl/mem/test.mem",ios::out);

    char c0, c1, c2, c3;

    if(!bin_file){
        cout<<"Failed to open "<<argv[1]<<" file!!!"<<endl;
        return 1;
    } else if(!mem_file) {
        cout<<"Failed to open "<<"/home/twccyoyo/riscv/Daicjou_2/aquila/ip_repo/aquila/hdl/mem/test.mem"<<" file!!!"<<endl;
        return 1;
    } else {
        while(bin_file.get(c0)){
            bin_file.get(c1);
            bin_file.get(c2);
            bin_file.get(c3);
            mem_file <<setw(2) << setfill('0') << hex << (int)(unsigned char)c3;
            mem_file <<setw(2) << setfill('0') << hex << (int)(unsigned char)c2;
            mem_file <<setw(2) << setfill('0') << hex << (int)(unsigned char)c1;
            mem_file <<setw(2) << setfill('0') << hex << (int)(unsigned char)c0;
            mem_file << "\n";
        }
        bin_file.close();
        mem_file.close();
    }
  }

  if (argc > 3 ) {
    if (argv[4][0] == '1')
      rv_test_enable = true;
    cout << "set rv_test_enable to " << (rv_test_enable ? "\"true\"" : "\"false\"") << endl;
  }

  top = new Vaquila_testharness("top");
#ifdef TRACE
  Verilated::traceEverOn(true);
  Vcdfp = new VerilatedVcdC;
  top->trace(Vcdfp, 99);
  Vcdfp->open("aquila_core.vcd");
#endif
  uint32_t entry_addr = 0x00000000;

  if(argc > 2 && argv[3][0] == '1'){
    elf_symbols = sim_mem_load_program(top->aquila_testharness->mock_ram, string(argv[1]), &entry_addr);
  }
  if (rv_test_enable) {
    if (elf_symbols.count("tohost")){
      tohost_addr = static_cast<uint32_t>(elf_symbols["tohost"]);
    } else {
      cerr << "no tohost symbols existed.!!!" << endl;
      return -1;
    }
  }

  top->rst_n = 0;
  cout << "entry_addr = " << "0x" << setfill('0') << setw(8) << right << hex << entry_addr << endl;
  top->main_memory_addr = entry_addr;
  //load_simple_asm();
  sim_mem_dump_memory(top->aquila_testharness->mock_ram, "dump.mem");
  for (int i = 0 ; i < 5 ; i ++){
    top->clk = 0;
    top->eval ();
    cpuTime += 5;
#ifdef TRACE
    Vcdfp->dump(cpuTime);
#endif
    top->clk = 1;
    top->eval ();
    cpuTime += 5;
#ifdef TRACE
    Vcdfp->dump(cpuTime);
#endif
  }
  top->rst_n = 1;

  uint32_t tohost_val;

  for (int i = 0 ; i < MAX_SIM_CYCLE ; i ++){
    top->clk = 0;
    top->eval ();
    cpuTime += 5;
#ifdef TRACE
    Vcdfp->dump(cpuTime);
#endif
    log_file << "#" << setfill('0') << setw(10) << right << i <<
      ":" << setfill('0') << setw(8) << right << hex << top->cur_instr_addr << endl;
    top->clk = 1;
    top->eval ();
    cpuTime += 5;
#ifdef TRACE
    Vcdfp->dump(cpuTime);
#endif
    if (rv_test_enable) {
#ifdef FENCE_ENABLE
      tohost_val = sim_mem_tohost_monitor(top->aquila_testharness->mock_ram, tohost_addr);
#else
      tohost_val = top->aquila_testharness->mock_uart_0->read_tohost();
#endif
      if (tohost_val != 0){
        if (tohost_val == 1)
          cout << "pass testcase: " << argv[1] << endl;
        else
          cout << "testcase #" << tohost_val << " failed !!!!!\ntestcase:" << argv[1] << endl;
        break;
      }
    }
  }
#ifdef TRACE
	Vcdfp->close();
  delete Vcdfp;
#endif
  delete top;

	if (rv_test_enable && tohost_val != 1)
    exit(-1);
  else
    exit(0);
}

void load_simple_asm()
{
//_start
  top->aquila_testharness->mock_ram->writeWord(0x00000000,0x04c0006f);
// trap_vector
  top->aquila_testharness->mock_ram->writeWord(0x00000004,0x34202f73);
  top->aquila_testharness->mock_ram->writeWord(0x00000008,0x00800f93);
  top->aquila_testharness->mock_ram->writeWord(0x0000000c,0x03ff0a63);
  top->aquila_testharness->mock_ram->writeWord(0x00000010,0x00900f93);
  top->aquila_testharness->mock_ram->writeWord(0x00000014,0x03ff0663);
  top->aquila_testharness->mock_ram->writeWord(0x00000018,0x00b00f93);
  top->aquila_testharness->mock_ram->writeWord(0x0000001c,0x03ff0263);
  top->aquila_testharness->mock_ram->writeWord(0x00000020,0x00000f17);
  top->aquila_testharness->mock_ram->writeWord(0x00000024,0xfe0f0f13);
  top->aquila_testharness->mock_ram->writeWord(0x00000028,0x000f0463);
  top->aquila_testharness->mock_ram->writeWord(0x0000002c,0x000f0067);
  top->aquila_testharness->mock_ram->writeWord(0x00000030,0x34202f73);
  top->aquila_testharness->mock_ram->writeWord(0x00000034,0x000f5463);
  top->aquila_testharness->mock_ram->writeWord(0x00000038,0x0040006f);
// handle exception
  top->aquila_testharness->mock_ram->writeWord(0x0000003c,0x5391e193);
  top->aquila_testharness->mock_ram->writeWord(0x00000040,0x00001f17);
  top->aquila_testharness->mock_ram->writeWord(0x00000044,0xfc3f2023);
  top->aquila_testharness->mock_ram->writeWord(0x00000048,0xff9ff06f);
//reset vector
  top->aquila_testharness->mock_ram->writeWord(0x0000004c,0xf1402573);
  top->aquila_testharness->mock_ram->writeWord(0x00000050,0x00051063);
  top->aquila_testharness->mock_ram->writeWord(0x00000054,0x00000297);
  top->aquila_testharness->mock_ram->writeWord(0x00000058,0x01028293);
  top->aquila_testharness->mock_ram->writeWord(0x0000005c,0x30529073);
  top->aquila_testharness->mock_ram->writeWord(0x00000060,0x18005073);
  top->aquila_testharness->mock_ram->writeWord(0x00000064,0x00000297);
  top->aquila_testharness->mock_ram->writeWord(0x00000068,0x02028293);
  top->aquila_testharness->mock_ram->writeWord(0x0000006c,0x30529073);
  top->aquila_testharness->mock_ram->writeWord(0x00000070,0x800002b7);
}


