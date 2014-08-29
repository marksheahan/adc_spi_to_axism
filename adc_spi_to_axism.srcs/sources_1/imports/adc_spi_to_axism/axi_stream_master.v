////////////////////////////////////////////////////////////////////////////
//-- (c) Copyright 2012 - 2013 Xilinx, Inc. All rights reserved.
//--
//-- This file contains confidential and proprietary information
//-- of Xilinx, Inc. and is protected under U.S. and
//-- international copyright and other intellectual property
//-- laws.
//--
//-- DISCLAIMER
//-- This disclaimer is not a license and does not grant any
//-- rights to the materials distributed herewith. Except as
//-- otherwise provided in a valid license issued to you by
//-- Xilinx, and to the maximum extent permitted by applicable
//-- law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
//-- WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
//-- AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
//-- BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
//-- INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
//-- (2) Xilinx shall not be liable (whether in contract or tort,
//-- including negligence, or under any other theory of
//-- liability) for any loss or damage of any kind or nature
//-- related to, arising under or in connection with these
//-- materials, including for any direct, or any indirect,
//-- special, incidental, or consequential loss or damage
//-- (including loss of data, profits, goodwill, or any type of
//-- loss or damage suffered as a result of any action brought
//-- by a third party) even if such damage or loss was
//-- reasonably foreseeable or Xilinx had been advised of the
//-- possibility of the same.
//--
//-- CRITICAL APPLICATIONS
//-- Xilinx products are not designed or intended to be fail-
//-- safe, or for use in any application requiring fail-safe
//-- performance, such as life-support or safety devices or
//-- systems, Class III medical devices, nuclear facilities,
//-- applications related to the deployment of airbags, or any
//-- other applications that could lead to death, personal
//-- injury, or severe property or environmental damage
//-- (individually and collectively, "Critical
//-- Applications"). Customer assumes the sole risk and
//-- liability of any use of Xilinx products in Critical
//-- Applications, subject only to applicable laws and
//-- regulations governing limitations on product liability.
//--
//-- THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
//-- PART OF THIS FILE AT ALL TIMES.
////////////////////////////////////////////////////////////////////////////
//
// AXI4-Stream interface example
//
// The purpose of this design is to provide a simple AXI4-Stream interface.
//
// The AXI4-Stream protocol is used as a standard interface to connect components that wish to
// exchange data. The interface can be used to connect a single master, that generates data, to a
// single slave, that receives data.
//
////////////////////////////////////////////////////////////////////////////////
`timescale 1ps/1ps

module adc_spi_to_axism #(
  ////////////////////////////////////////////////////////////////////////////////
  // Width of S_AXIS address bus. The slave accepts the read and write addresses
  // of width C_M_AXIS_TDATA_NUM_BYTES.
  parameter integer C_M_AXIS_TDATA_NUM_BYTES = 2,
  parameter integer C_PACKET_LENGTH          = 1,
  parameter integer ADC_RES_BITS             = 16
) (
  /////////////////// extra inputs be Mark Sheahan
  // 
  input wire ADC_SPI_CNV,   // connected to a GPIO from the FPGA, then to CNV of ADCs
  input wire ADC_SPI_SCK,   // adc serial clock. Runs in 16 period bursts whilst CNV is low.
                            // this is used to clock in serial data from a GPIO. Once CNV is
                            // asserted, this initiates the AXI transaction.
  input wire ADC_SPI_SDO,   // ADC serial data out. GPIO input from the ADC    
  input wire ADC_SPI_RESETN, // Inverse reset for spi logic used here                      
  ////////////////////////////////////////////////////////////////////////////////
  // Global ports
  input wire AXIS_ACLK,
  input wire AXIS_ARESETN,
  ////////////////////////////////////////////////////////////////////////////////
  // Master Stream Ports
  // TVALID indicates that the master is driving a valid transfer.
  // A transfer takes place when both TVALID and TREADY are asserted.
  output wire M_AXIS_TVALID,
  ////////////////////////////////////////////////////////////////////////////////
  // TDATA is the primary payload that is used to provide the data
  // that is passing across the interface from the master
  output wire [(C_M_AXIS_TDATA_NUM_BYTES*8)-1:0] M_AXIS_TDATA,
  ////////////////////////////////////////////////////////////////////////////////
  // TSTRB is the byte qualifier that indicates whether the content
  // of the associated byte of TDATA is processed as a data byte or
  // a position byte.
  output wire [C_M_AXIS_TDATA_NUM_BYTES-1:0] M_AXIS_TSTRB,
  ////////////////////////////////////////////////////////////////////////////////
  // TLAST indicates the boundary of a packet.
  output wire M_AXIS_TLAST,
  ////////////////////////////////////////////////////////////////////////////////
  // TREADY indicates that the slave can accept a transfer
  // in thecurrent cycle.
  input wire M_AXIS_TREADY
);
////////////////////////////////////////////////////////////////////////////////
// function called clogb2 that returns an integer which has the
// value of the ceiling of the log base 2.
function integer clogb2 (input integer bd);
integer bit_depth;
  begin
    bit_depth = bd;
    for(clogb2=0; bit_depth>0; clogb2=clogb2+1)
      bit_depth = bit_depth >> 1;
  end
endfunction

localparam LP_PACKET_COUNTER_WIDTH  = clogb2(C_PACKET_LENGTH);

reg [LP_PACKET_COUNTER_WIDTH-1:0]  packet_counter;

localparam IDLE    = 1'b0,
           SEND    = 1'b1;

reg cnv_edge;
reg cnv_edge_last;
reg state;
reg [(ADC_RES_BITS-1):0] adc_sample;
reg [(C_M_AXIS_TDATA_NUM_BYTES*8)-1:0] m_axis_tdata;
reg m_axis_tvalid;
reg m_axis_tlast;

always@(posedge ADC_SPI_CNV) begin
   if(!ADC_SPI_RESETN) begin
       cnv_edge <= 0;
   end else begin
       cnv_edge = !cnv_edge;
   end 
end

always@(posedge ADC_SPI_SCK) begin
    if(!ADC_SPI_RESETN) begin
        adc_sample <= {ADC_RES_BITS{1'b0}};
    end else begin
        adc_sample <= {adc_sample[14:0], ADC_SPI_SDO};
    end 
end

always@(posedge AXIS_ACLK) begin
  if(!AXIS_ARESETN) begin
    cnv_edge_last <= 0;
    packet_counter <= {LP_PACKET_COUNTER_WIDTH{1'b0}};
    state <= IDLE;
    m_axis_tvalid <= 1'b0;
    m_axis_tlast <= 1'b0;
    m_axis_tdata <= {C_M_AXIS_TDATA_NUM_BYTES*8{1'b0}};
  end else begin
    case (state)
      IDLE: begin
        packet_counter <= {LP_PACKET_COUNTER_WIDTH{1'b0}};
        state <= IDLE;
        m_axis_tvalid <= 1'b0;
        m_axis_tlast <= 1'b0;
        if (cnv_edge != cnv_edge_last) begin
            state <= SEND;
        end
      end
      SEND: begin
        m_axis_tvalid <= 1'b1;
        m_axis_tlast <= 1'b0;
        state <= SEND;
        if (packet_counter == C_PACKET_LENGTH) begin
          m_axis_tlast <= 1'b1;
          state <= IDLE;
        end else if (M_AXIS_TVALID && M_AXIS_TREADY) begin
          packet_counter <= packet_counter + 1;
          m_axis_tdata <= adc_sample;
        end
      end
      default : begin
        packet_counter <= {LP_PACKET_COUNTER_WIDTH{1'b0}};
        state <= IDLE;
        m_axis_tvalid <= 1'b0;
        m_axis_tlast <= 1'b0;
        m_axis_tdata <= {C_M_AXIS_TDATA_NUM_BYTES*8{1'b0}};
      end
    endcase
    cnv_edge_last <= cnv_edge;
  end
end

assign M_AXIS_TSTRB = {C_M_AXIS_TDATA_NUM_BYTES{1'b1}};
assign M_AXIS_TVALID = m_axis_tvalid;
assign M_AXIS_TLAST = m_axis_tlast;
assign M_AXIS_TDATA = m_axis_tdata;

endmodule
