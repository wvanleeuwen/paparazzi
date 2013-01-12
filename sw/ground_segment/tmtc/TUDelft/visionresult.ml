(*
* $Id: compass.ml 3941 2009-08-15 10:18:18Z hecto $
*
* Compass display for a manned vehicle
*
* Copyright (C) 2004-2009 ENAC, Pascal Brisset, Antoine Drouin
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

open Printf
open Latlong

module Tm_Pprz = Pprz.Messages (struct let name = "telemetry" end)

let width = 300
let height = 200
let background = `NAME "grey"
let fore = `BLACK

let data = [128; 100; 140; 230; 90; 80; 70; 100; 120; 110; 10; 15;]

let get_array_length = fun value ->
  match value with
    Pprz.Int x -> 1
  | Pprz.Array x ->
      Array.length x
  | _ -> invalid_arg "Pprz.int_assoc"


let int_of_value = fun value ->
  match value with
    Pprz.Int x -> x
  | Pprz.Int32 x ->
      let i = Int32.to_int x in
      if Int32.compare x (Int32.of_int i) <> 0 then
	failwith "Pprz.int_assoc: Int32 too large to be converted into an int";
      i
  | _ -> invalid_arg "Vision.int_of_value"


let get_pprz_element = fun value nr ->
  match value with
    Pprz.Array x ->
            x.(nr)
  | _ -> invalid_arg "Vision.getelement"



let draw = fun (da_object:Gtk_tools.pixmap_in_drawin_area) payload ->
  let nrofcolums = get_array_length payload in
  (* Drawing Area *)
  let da = da_object#drawing_area in
  let {Gtk.width=width; height=height} = da#misc#allocation in
  let dr = da_object#get_pixmap () in
  (* Background *)
  dr#set_foreground background;
  dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();
  dr#set_foreground fore;

  let colwidth = width / nrofcolums in

(*
  let i = ref 0 in
  List.iter (fun x ->
    let pos = !i * colwidth in
    let v = x * height / 256 in
    let dy = height - v in
    i := !i +1;
    dr#rectangle ~x:pos ~y:dy ~width:30 ~height:v ~filled:true ())
    data;
  *)

  for i = 0 to (nrofcolums-1) do (
    let ppval = get_pprz_element payload i in
    let valu = int_of_value ppval in
    let pos = i * colwidth in
    let v = valu * height / 256 in
    let dy = height - v in
    dr#rectangle ~x:pos ~y:dy ~width:30 ~height:v ~filled:true ()
  ) done;

  (new GDraw.drawable da#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap

(*********************** Main ************************************************)
let _ =
  let ivy_bus = ref "127.255.255.255:2010" in

  Arg.parse
    [ "-b", Arg.String (fun x -> ivy_bus := x), "Bus\tDefault is 127.255.255.255:2010"]
    (fun x -> prerr_endline ("WARNING: don't do anything with "^x))
    "Usage: ";

  (** Connect to the Ivy bus *)
  Ivy.init "Paparazzi Vision Results" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  (** Open the window *)
  let icon = GdkPixbuf.from_file Env.icon_file in
  let window = GWindow.window ~icon ~title:"Vision Results" () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);

  let da = new Gtk_tools.pixmap_in_drawin_area ~width ~height ~packing:window#add () in
  da#drawing_area#misc#realize ();

  (* Empty Screen *)


  (* Listening messages *)
  let get_payload = fun _ values ->
    let payload = Pprz.assoc "values" values in
    draw da payload in
  ignore (Tm_Pprz.message_bind "PAYLOAD" get_payload);

(*  get_payload (["mode", 0]); *)


  (** Start the main loop *)
  window#show ();
  GMain.main ()

