--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
-- 
-- Authors:
--     Ondrej Ille <ondrej.ille@gmail.com>
--     Martin Jerabek <martin.jerabek01@gmail.com>
-- 
-- Project advisors: 
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 
-- Department of Measurement         (http://meas.fel.cvut.cz/)
-- Faculty of Electrical Engineering (http://www.fel.cvut.cz)
-- Czech Technical University        (http://www.cvut.cz/)
-- 
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to deal in the Component without restriction, including without limitation
-- the rights to use, copy, modify, merge, publish, distribute, sublicense,
-- and/or sell copies of the Component, and to permit persons to whom the
-- Component is furnished to do so, subject to the following conditions:
-- 
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Component.
-- 
-- THE COMPONENT IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE COMPONENT OR THE USE OR OTHER DEALINGS
-- IN THE COMPONENT.
-- 
-- The CAN protocol is developed by Robert Bosch GmbH and protected by patents.
-- Anybody who wants to implement this IP core on silicon has to obtain a CAN
-- protocol license from Bosch.
-- 
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Purpose:
--    Test of AHB adapter.
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

architecture ahb_unit_test of CAN_test is
    
    ---------------------------------------------------------------------------
    -- CTU CAN Interface 
    ---------------------------------------------------------------------------
    signal hresetn          :  std_logic;
    signal hclk             :  std_logic;
    signal haddr            :  std_logic_vector(31 downto 0);
    signal hwdata           :  std_logic_vector(31 downto 0);
    signal hsel             :  std_logic;
    signal hwrite           :  std_logic;
    signal hsize            :  std_logic_vector(2 downto 0);
    signal hburst           :  std_logic_vector(2 downto 0);
    signal hprot            :  std_logic_vector(3 downto 0);
    signal htrans           :  std_logic_vector(1 downto 0);
    signal hmastlock        :  std_logic;
    signal hready           :  std_logic;
    signal hreadyout        :  std_logic;
    signal hresp            :  std_logic;
    signal hrdata           :  std_logic_vector(31 downto 0);
   
    signal can_tx           :  std_logic;
    signal can_rx           :  std_logic := '1';
   
    signal timestamp        :  std_logic_vector(63 downto 0) := (OTHERS => '0');
    signal int              :  std_logic;

    ---------------------------------------------------------------------------
    -- AHB driver
    ---------------------------------------------------------------------------
    type t_ahb_access is record
        hwrite          : std_logic;
        hsize           : std_logic_vector(2 downto 0);
        address         : std_logic_vector(31 downto 0);
        write_data      : std_logic_vector(31 downto 0);
        exp_read_data   : std_logic_vector(31 downto 0);
    end record;
    
    type t_ahb_access_fifo is array (integer range <>) of t_ahb_access;
    
    ---------------------------------------------------------------------------
    -- AHB queue of Memory accesses
    ---------------------------------------------------------------------------
    type t_ahb_access_queue is protected
        procedure push(acc : t_ahb_access);
        procedure pop(acc : t_ahb_acces);
        procedure execute;
    end protected ahb_access_queue;
    
    type t_ahb_access_queue is protected body
    
        variable transaction_count : natural := 0;
        variable wp, rp : natural range 0 to 7 := 0;
        variable executing : boolean := false;
        variable acess_fifo : t_ahb_access_fifo(0 to 7);

        procedure push(acc : t_ahb_access) is
        begin
            if (transaction_count = 8) then
                error("AHB Transaction FIFO Full -> Can't Push!");
                return;
            end if;
            
            access_fifo[wp] := acc;
            wait for 0 ns;
            transaction_count := transaction_count + 1;
            wp := (wp + 1) mod wp'high;
        end procedure;

        procedure pop(acc : out t_ahb_acces) is
        begin
            if (transaction_count = 0) then
                error("AHB Transaction FIFO Empty -> nothing to POP!");
            end if;
            
            acc := access_fifo[rp];
            wait for 0 ns;
            rp := (rp + 1) mod rp'high;
            
            transaction_count := transaction_count - 1;
            if (transaction_count = 0) then
                stop_executing;
            end if;
        end procedure;
        
        procedure start_executing is
        begin
            if (transaction_count = 0) then
                warning("No transactions to be executed! Finish.");
                return;
            end if;
            
            if (executing) then
                warning("Already executing -> No effect");
                return;
            end if;
            
            executing := true;
        end procedure;
        
        procedure stop_executing is
        begin
            executing := false;
        end procedure;

        impure function is_executing return boolean is
        begin
            return executing;
        end function;

    end protected body t_ahb_access_queue;

    ---------------------------------------------------------------------------
    -- Queue itself
    ---------------------------------------------------------------------------
    shared variable ahb_access_queue : t_ahb_access_queue;

    ---------------------------------------------------------------------------
    -- Wrapper procedures
    ---------------------------------------------------------------------------
    function ahb_check_alignment(
        address          : std_logic_vector;
        size             : natural range 1 to 4
    ) return boolean is
    begin
        case (address(1 downto 0)) is
        when "00" =>
            return true;
        
        when "01" =>
            if (size = 1) then
                return true;
            end if;
        
        when "10" =>
            if (size = 1 or size = 2) then
                return true;
            end if;
        
        when "11" =>
            if (size = 1) then
                return true;
            end if;
        
        when others =>
        end case;
        
        return false;
    end function;

    
    procedure ahb_write(
        address          : std_logic_vector;
        write_data       : std_logic_vector;
        size             : natural range 1 to 4
    )is
        variable acc : t_ahb_access;
    begin
        if (ahb_check_alignment(address, size) = false) then
            error("Unaligned AHB Write!");
        end if;
        
        acc.address := address;
        acc.hwrite := '1';
        acc.write_data := write_data;

        t_ahb_access_queue.push(acc);        
    end procedure;


    procedure ahb_read(
        address          : std_logic_vector;
        exp_read_data    : std_logic_vector;
        size             : natural range 1 to 4
    )is
        variable acc : t_ahb_access;
    begin
        if (ahb_check_alignment(address, size) = false) then
            error("Unaligned AHB Read!");
        end if;

        acc.address := address;
        acc.hwrite := '0';
        acc.exp_read_data := exp_read_data;
        t_ahb_access_queue.push(acc);
    end procedure;

    


begin

    can_top_ahb_inst : can_top_ahb
    port map(
        hresetn          => hresetn,
        hclk             => hclk,
        haddr            => haddr,
        hwdata           => hwdata,
        hsel             => hsel,
        hwrite           => hwrite,
        hsize            => hsize,
        hburst           => hburst,
        hprot            => hprot,
        htrans           => htrans,
        hmastlock        => hmastlock,
        hready           => hready,
        hreadyout        => hreadyout,
        hresp            => hresp,
        hrdata           => hrdata,
        
        can_tx           => can_tx,
        can_rx           => can_rx,
        
        timestamp        => timestamp,

        int              => int     
    );

    ---------------------------------------------------------------------------
    -- 
    ---------------------------------------------------------------------------
    ahb_driver_proc : process
        procedure process_ahb_cycle is
            variable acc : t_ahb_access;
        begin
            if () then
                
            
            
            -- If slave signals HREADY low, hold on!
            if (hready = '1') then
                    
            end if;
            
            
        end procedure;
    begin
        wait until rising_edge(clk_sys);
        
        if (ahb_access_queue.is_executing) then
            if (ahb_access_queue.transaction_count = 0) then
                error("Queue should not be executing without transactions!");
            end if;
            
            process_ahb_cycle;
        else
            hsel <= '0';
            haddr <= (OTHERS => '0');
            hwdata <= (OTHERS => '0');
            hsize <= (OTHERS => '0');
            hprot <= (OTHERS => '0');
        end if;        
    end process;
    
    -- Connect hready directly, we have only single slave here!
    hready <= hreadyout;

    ---------------------------------------------------------------------------
    -- 
    ---------------------------------------------------------------------------
    dut_driver_proc : process
    begin
        hresetn <= '0';
        wait for 40 ns;
        hresetn <= '1';
        wait for 40 ns;
        
    end process;

end architecture;