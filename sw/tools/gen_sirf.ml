(*
 * XML preprocessing for SIRF protocol
 *)

open Printf

let out = stdout

let sizeof = function
    "U4" | "I4" -> 4
  | "U2" | "I2" -> 2
  | "U1" | "I1" -> 1
  | x -> failwith (sprintf "sizeof: unknown format '%s'" x)

let (+=) = fun r x -> r := !r + x

let c_type = fun format ->
  match format with
    "I2" -> "int16_t"
  | "I4" -> "int32_t"
  | "U2" -> "uint16_t"
  | "U4" -> "uint32_t"
  | "U1" -> "uint8_t"
  | "I1" -> "int8_t"
  | _ -> failwith (sprintf "Gen_sirf.c_type: unknown format '%s'" format)

let get_at = fun offset format block_size ->
  let t = c_type format in
  let block_offset =
    if block_size = 0 then "" else sprintf "+%d*_sirf_block" block_size in
  match format with
     "U4" | "I4" -> sprintf "(%s)(*((uint8_t*)_sirf_payload+%d%s)|*((uint8_t*)_sirf_payload+1+%d%s)<<8|((%s)*((uint8_t*)_sirf_payload+2+%d%s))<<16|((%s)*((uint8_t*)_sirf_payload+3+%d%s))<<24)" t offset block_offset offset block_offset t offset block_offset t offset block_offset
   | "U2" | "I2" -> sprintf "(%s)(*((uint8_t*)_sirf_payload+%d%s)|*((uint8_t*)_sirf_payload+1+%d%s)<<8)" t offset block_offset offset block_offset
   | "U1" | "I1" -> sprintf "(%s)(*((uint8_t*)_sirf_payload+%d%s))" t offset block_offset
   | _ -> failwith (sprintf "Gen_sirf.c_type: unknown format '%s'" format)

let define = fun x y ->
  fprintf out "#define %s %s\n" x y

exception Length_error of Xml.xml*int*int


let parse_message = fun m ->
  let msg_name = Xml.attrib m "name" in

  fprintf out "\n";
  let msg_id = sprintf "SIRF_%s_ID" msg_name in
  define msg_id (Xml.attrib m "ID");

  let field_name = fun f -> ExtXml.attrib f "name" in
  let format = fun f -> Xml.attrib f "format" in

  let offset = ref 0 in
  let rec gen_access_macro = fun block_size f ->
    match Xml.tag f with
      "field" ->
	let fn = field_name f
	and fmt = format f  in
	let block_no = if block_size = 0 then "" else ",_sirf_block" in
	define (sprintf "SIRF_%s_%s(_sirf_payload%s)" msg_name fn block_no) (get_at !offset fmt block_size);
	offset += sizeof fmt
    | x -> failwith ("Unexpected field: " ^ x)
  in

  List.iter (gen_access_macro 0) (Xml.children m);
  begin
    try
      let l = int_of_string (Xml.attrib m "length") in
      if l <> !offset then raise (Length_error (m, l, !offset))
    with
      Xml.No_attribute("length") -> () (** Undefined length authorized *)
  end;

  (** Generating send function *)
  let param_name = fun f -> String.lowercase (field_name f) in
  let rec param_names = fun f r ->
    if Xml.tag f = "field" then
      param_name f :: r
    else
      List.fold_right param_names (Xml.children f) r in
  let param_type = fun f -> c_type (format f) in
  fprintf out "\n#define SirfSend_%s_%s(" class_name msg_name;
  fprintf out "%s" (String.concat "," (param_names m []));
  fprintf out ") { \\\n";
  fprintf out "  SirfHeader(SIRF_%s_ID, %s, %d);\\\n" class_name msg_id !offset;
  let rec send_one_field = fun f ->
    match Xml.tag f with
      "field" ->
	let s = sizeof (format f) in
	let p = param_name f in
	let t = param_type f in
	fprintf out "  %s _%s = %s; SirfSend%dByAddr((uint8_t*)&_%s);\\\n" t p p s p
    | _ -> assert (false) in
  List.iter send_one_field (Xml.children m);
  fprintf out "  SIRFTrailer();\\\n";
  fprintf out "}\n"


let _ =
  if Array.length Sys.argv <> 2 then begin
    failwith (sprintf "Usage: %s <.xml sirf protocol file>" Sys.argv.(0))
  end;
  let xml_file = Sys.argv.(1) in
  try
    let xml = Xml.parse_file xml_file in
    fprintf out "/* Generated from %s */\n" xml_file;
    fprintf out "/* Please DO NOT EDIT */\n\n";

    define "SIRF_SYNC1" "0xA0";
    define "SIRF_SYNC2" "0xA2";

    List.iter parse_message (Xml.children xml)
  with
    Xml.Error (em, ep) ->
      let l = Xml.line ep
      and c1, c2 = Xml.range ep in
      fprintf stderr "File \"%s\", line %d, characters %d-%d:\n" xml_file l c1 c2;
      fprintf stderr "%s\n" (Xml.error_msg em);
      exit 1
  | Length_error (m, l1, l2) ->
      fprintf stderr "File \"%s\", inconsistent length: %d expected, %d found from fields in message:\n %s\n" xml_file l1 l2 (Xml.to_string_fmt m);
      exit 1
  | Dtd.Check_error e ->
      fprintf stderr "File \"%s\", DTD check error: %s\n" xml_file (Dtd.check_error e)
  | Dtd.Prove_error e ->
      fprintf stderr "\nFile \"%s\", DTD check error: %s\n\n" xml_file (Dtd.prove_error e)
