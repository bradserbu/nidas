var periodic = {

  cnt: 1,

  UpdateClocks: function() {
    var resp = GetClocks();
    if (resp)
      for (name in resp) {
        if (document.getElementById(name))
          document.getElementById(name).innerHTML =
            document.getElementById(name).label+' ('+resp[name]+')';
      }
  },

  loop: function() {
    periodic.UpdateClocks();
    if (++periodic.cnt > 1) {
      periodic.cnt=1;
      if (selectedDsm)
        recvStat( GetStatus[selectedDsm]() );
    }
    if (is_periodic)
      setTimeout('periodic.loop()',1000);
  }
}
window.onload=periodic.loop;
