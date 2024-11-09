`include "uvm_macros.svh"
import uvm_pkg::*;
typedef enum {IDLE,BUSY,NONSEQ,SEQ} trans_e;
typedef enum {SINGLE,INCR,INCR4,WRAP4,INCR8,WRAP8,INCR16,WRAP16} burst_e;

interface intf();
  logic HCLK;
  logic HRESETn;
  logic [7:0] HADDR;
  logic [31:0] HWDATA;
  logic HWRITE;
  trans_e HTRANS;
  burst_e HBURST;
  logic [1:0] HSIZE;
  logic [31:0] HRDATA;
  logic HREADY;
  logic HRESP;
  clocking cb@(posedge HCLK);
    input HRDATA;
    input HREADY;
    input HRESP;
    output HADDR;
    output HWDATA;
    output HWRITE;
    output HTRANS;
    output HBURST;
    output HSIZE;
  endclocking
  modport RTL(
    input HCLK,
    input HRESETn,
    input HADDR,
    input HWDATA,
    input HWRITE,
    input HTRANS,
    input HBURST,
    input HSIZE,
    output HRDATA,
    output HREADY,
    output HRESP   
  );
endinterface

class trans extends uvm_sequence_item;
  rand logic [7:0] HADDR;
  rand logic [31:0] HWDATA[];
  rand logic HWRITE;
  rand burst_e HBURST;
  rand logic [1:0] HSIZE;
  rand logic [31:0] HRDATA[];
  rand int num_cycles;
  `uvm_object_utils_begin(trans)
     `uvm_field_int(HADDR, UVM_ALL_ON)
     `uvm_field_array_int(HWDATA, UVM_ALL_ON)
     `uvm_field_int(HWRITE, UVM_ALL_ON)
     `uvm_field_enum(burst_e,HBURST, UVM_ALL_ON)
     `uvm_field_int(HSIZE, UVM_ALL_ON)
     `uvm_field_array_int(HRDATA, UVM_ALL_ON)
     `uvm_field_int(num_cycles, UVM_ALL_ON)
  `uvm_object_utils_end
  constraint const_c{
    HSIZE inside {[0:2]};
    HSIZE == 1 -> HADDR[0] == 0;
    HSIZE == 2 -> HADDR[1:0] == '0;
    num_cycles dist {1:/1,[2:3]:/1,4:/1,[5:7]:/1,8:/1,[9:15]:/1,16:/1,[17:32]:/1};
    if(num_cycles == 1) HBURST == SINGLE;
    else if(num_cycles == 4) HBURST inside {INCR4,WRAP4};
    else if(num_cycles == 8) HBURST inside {INCR8,WRAP8};
    else if(num_cycles == 16) HBURST inside {INCR16,WRAP16};
    else HBURST == INCR;
    HWDATA.size == num_cycles;
    HRDATA.size == num_cycles;
    HWRITE == 0 -> foreach(HWDATA[i]) { HWDATA[i] == '0;}
    HWRITE == 1 -> foreach(HRDATA[i]) { HRDATA[i] == '0;}      
    solve HSIZE before HADDR;
    solve num_cycles before HBURST;
    solve HWRITE before num_cycles;
    solve num_cycles before HWDATA;
    solve num_cycles before HRDATA;
  }
  function new(string name="");
    super.new(name);
  endfunction
endclass

class test extends uvm_test;
  bit [3:0] num_trans;
  rand trans t;
  trans t_q[$];
  virtual intf my_intf;
  `uvm_component_utils(test)
  function new(string name="", uvm_component parent=null);
    super.new(name,parent);
  endfunction
  virtual function void build_phase(uvm_phase phase);
    if(!uvm_config_db#(virtual intf)::get(this,"","INTF",my_intf))
      `uvm_fatal(get_type_name,"INTF object not found")
  endfunction
  virtual function void end_of_elaboration_phase(uvm_phase phase);
    uvm_phase main_phase = phase.find_by_name("main",0);
    main_phase.phase_done.set_drain_time(this, 100ns);
  endfunction
  virtual task main_phase(uvm_phase phase);
    phase.raise_objection(this);
    fork 
    begin
      drive_trans();
    end
    join_none
    std::randomize(num_trans);
    for(int i=0; i<num_trans; i+=1) begin
      t = trans::type_id::create("t");
      t.randomize();
      t_q.push_back(t);
    end
    phase.drop_objection(this);
  endtask
  virtual function void phase_ready_to_end(uvm_phase phase);
    if(phase.get_name() == "main") begin
      if(t_q.size != 0) begin
        fork
          begin
            phase.raise_objection(this);
            wait_for_drive(phase);
          end
        join_none
      end
    end
  endfunction
  task wait_for_drive(uvm_phase phase); 
    wait(t_q.size == 0);
    phase.drop_objection(this);
  endtask
  task drive_trans();
    trans t1;
    forever begin
      if(t_q.size > 0) begin
        t1 = t_q.pop_front();
        drive_addr_phase(t1);
      end
      else begin
        drive_idle();
      end
    end
  endtask
  task drive_addr_phase(trans t);
    bit [31:0] wdata;
    for(int i=0; i<t.num_cycles; i+=1) begin
      @my_intf.cb;
      if(my_intf.cb.HREADY==1 || i==0) begin
         my_intf.cb.HADDR <= get_addr(t.HADDR,t.HSIZE,t.HBURST,i);
         my_intf.cb.HWRITE <= t.HWRITE;
         my_intf.cb.HSIZE <= t.HSIZE;
         my_intf.cb.HBURST <= t.HBURST;        
         if(i==0) begin
           my_intf.cb.HTRANS <= NONSEQ;
         end else begin
           my_intf.cb.HTRANS <= SEQ;        
         end
         wdata = t.HWDATA[i];
         fork begin
           drive_data_phase(t.HWRITE,wdata);
         end
         join_none
      end
      else begin
        i-=1;
      end
    end
  endtask
  function bit [31:0] get_addr(bit [31:0] addr, bit [1:0] size, burst_e burst, int count);
    get_addr = addr + count*(1<<size);
    if(burst == WRAP4) begin
      get_addr = get_addr%((1<<size)*4);
    end
    else if(burst == WRAP8) begin
      get_addr = get_addr%((1<<size)*8);
    end
    else if(burst == WRAP16) begin
      get_addr = get_addr%((1<<size)*16);
    end
    return get_addr;
  endfunction
  task drive_data_phase(bit write, bit [31:0] wdata);
    @my_intf.cb;
    if(write == 1) begin
      my_intf.cb.HWDATA <= wdata;
    end
  endtask
  task drive_idle();
    @my_intf.cb;
    my_intf.cb.HTRANS <= IDLE;
  endtask
endclass

module top;
  
  intf my_intf();
  
  initial begin
    my_intf.HCLK <= 0;
    my_intf.HREADY <= 1; //Hack
    uvm_config_db#(virtual intf)::set(null,"*", "INTF", my_intf);
    forever begin
      #1 my_intf.HCLK <= ~ my_intf.HCLK;
    end
  end
  
  initial begin
    run_test("test");
  end
  
  initial begin
    $monitor("HADDR=%0h, HWRITE=%0h, HWDATA=%0h, HTRANS=%s, HSIZE=%0h, HBURST=%s, @time=%0t", my_intf.HADDR, my_intf.HWRITE, my_intf.HWDATA, my_intf.HTRANS.name, my_intf.HSIZE, my_intf.HBURST.name, $time);
  end
  
endmodule
