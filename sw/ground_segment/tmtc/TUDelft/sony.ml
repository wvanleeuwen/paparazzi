(*
 * $Id: settings.ml 6184 2010-10-19 15:43:01Z gautier $
 *
 * Multi aircraft settings handler
 *
 * Copyright (C) 2007 ENAC, Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *)

module DL = Pprz.Messages(struct let name = "datalink" end)
module 	Datalink_Pprz = Pprz.Messages(struct let name = "ground" end)
module Tele_Pprz = Pprz.Messages(struct let name = "telemetry" end)

let (//) = Filename.concat
let conf_dir = Env.paparazzi_home // "conf"
let conf_xml = Xml.parse_file (conf_dir // "conf.xml")
let my_id = "sonyinterface"

open Printf
let _ =
  let ivy_bus = ref "127.255.255.255:2010" in
  let host = ref "10.31.1.98"
  and port = ref 4242
  and ac_id = ref "6" in

  let options = [
    "-b", Arg.Set_string ivy_bus, (sprintf "<ivy bus> Default is %s" !ivy_bus);
    "-h", Arg.Set_string host, (sprintf "<remote host> Default is %s" !host);
    "-id", Arg.Set_string ac_id , (sprintf "<id> Default is %s" !host);
    "-p", Arg.Set_int port, (sprintf "<remote port> Default is %d" !port)
  ] in
  Arg.parse
    options
    (fun x -> fprintf stderr "Warning: Discarding '%s'" x)
    "Usage: ";

  (** Connect to the Ivy bus *)
  Ivy.init "Paparazzi Payload Control" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  (** Open the window container with its notebook*)
  let icon = GdkPixbuf.from_file Env.icon_file in
  let window = GWindow.window ~icon ~width:125 ~height:220 ~allow_shrink:true ~title:"PayLoad" () in
(*  ignore (window#gdk_window_set_back_pixmap icon);
  ignore (window#gtk_window_set_title "SONY");
  let img = GImage ~icon ~width:20 ~height:20 ~packing:window#pack () in *)



  (** Callback Function **)

  let send_command = fun code ->
    DL.message_send my_id "PAYLOAD_COMMAND"
      ["ac_id", Pprz.String !ac_id; "command", Pprz.Int code] in

(*

DL.message_send my_id "BLOCK"
      ["block_id", Pprz.Int code; "ac_id", Pprz.Int 22] in

    DL.message_send my_id "PAYLOAD_COMMAND"
      ["ac_id", Pprz.String ac_id; "command", Pprz.Int code] in
*)
(*    Datalink_Pprz.message_send my_id "JUMP_TO_BLOCK" [ "ac_id", Pprz.String "22"; "block_id", Pprz.Int code  ];
*)


  (** Make Buttons **)
  let vbox = GPack.vbox ~packing:window#add () in

  let hbox = GPack.hbox ~packing:vbox#pack () in
  let _O =  GButton.button ~label:"On/Off" ~packing:hbox#pack () in
  ignore (_O#connect#pressed (fun () -> send_command 111));
  let _P =  GButton.button ~label:"Play" ~packing:hbox#pack () in
  ignore (_P#connect#pressed (fun () -> send_command 112));

  let hbox = GPack.hbox ~packing:vbox#pack () in
  let _H =  GButton.button ~label:"Hold" ~packing:hbox#pack () in
  ignore (_H#connect#pressed (fun () -> send_command 13));
  let _S =  GButton.button ~label:"Shoot" ~packing:hbox#pack () in
  ignore (_S#connect#pressed (fun () -> send_command 32));

  let hbox = GPack.hbox ~packing:vbox#pack () in
  let _T =  GButton.button ~label:"T" ~packing:hbox#pack () in
  ignore (_T#connect#pressed (fun () -> send_command 116));
  let _W =  GButton.button ~label:"W" ~packing:hbox#pack () in
  ignore (_W#connect#pressed (fun () -> send_command 119));

  let hbox = GPack.hbox ~packing:vbox#pack () in
  let _L =  GButton.button ~label:"L" ~packing:hbox#pack () in
  ignore (_L#connect#pressed (fun () -> send_command 108));
  let _R =  GButton.button ~label:"R" ~packing:hbox#pack () in
  ignore (_R#connect#pressed (fun () -> send_command 114));
  let _C =  GButton.button ~label:"C" ~packing:hbox#pack () in
  ignore (_C#connect#pressed (fun () -> send_command 99));
  let _U =  GButton.button ~label:"U" ~packing:hbox#pack () in
  ignore (_U#connect#pressed (fun () -> send_command 117));
  let _D =  GButton.button ~label:"D" ~packing:hbox#pack () in
  ignore (_D#connect#pressed (fun () -> send_command 100));


(*
*)
(*
  let pix = GdkPixbuf.from_file (Env.paparazzi_home // "sw/ground_segment/tmtc/TUDelft/sony.jpg") in
  let pxm, _ = GdkPixbuf.create_pixmap pix in

  let da = new GMisc.drawing_area ~packing:window#add () in
  da#drawing_area#misc#realize (); *)
(*  let dw = new GDraw.drawable da#misc#window in *)
(*  dw#put_pixmap ~x:0 ~y:0 pxm#pixmap;
GMisc.drawing_area ?width ?height ~show:true ?packing ()


  (new GDraw.drawable da#misc#window)#put_pixmap ~x:0 ~y:0 pm#pixmap ;
 *)

  let hbox = GPack.hbox ~packing:vbox#pack () in
  let _M =  GButton.button ~label:"Menu" ~packing:hbox#pack () in
  ignore (_M#connect#pressed (fun () -> send_command 109));
  let _H =  GButton.button ~label:"Home" ~packing:hbox#pack () in
  ignore (_H#connect#pressed (fun () -> send_command 104));
  let _V =  GButton.button ~label:"VidOn" ~packing:hbox#pack () in
  ignore (_V#connect#pressed (fun () -> send_command 86));
  let _v =  GButton.button ~label:"VidOff" ~packing:hbox#pack () in
  ignore (_v#connect#pressed (fun () -> send_command 118));

  let hbox = GPack.hbox ~packing:vbox#pack () in
  let _0 =  GButton.button ~label:"Auto" ~packing:hbox#pack () in
  ignore (_0#connect#pressed (fun () -> send_command 48));
  let _1 =  GButton.button ~label:"P" ~packing:hbox#pack () in
  ignore (_1#connect#pressed (fun () -> send_command 49));
  let _2 =  GButton.button ~label:"Video" ~packing:hbox#pack () in
  ignore (_2#connect#pressed (fun () -> send_command 50));

  let hbox = GPack.hbox ~packing:vbox#pack () in
  let _5 =  GButton.button ~label:"Land" ~packing:hbox#pack () in
  ignore (_5#connect#pressed (fun () -> send_command 53));
  let _7 =  GButton.button ~label:"ISO" ~packing:hbox#pack () in
  ignore (_7#connect#pressed (fun () -> send_command 55));
  let _9 =  GButton.button ~label:"Easy" ~packing:hbox#pack () in
  ignore (_9#connect#pressed (fun () -> send_command 57));



  (** Start the main loop *)
  window#show ();
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done

