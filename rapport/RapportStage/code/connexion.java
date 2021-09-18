public int connexion(){
    if(master == null) {
    	try {
    		master = new ModbusTCPMaster(addr);
    		master.connect();
    		requete.connexion();
    	} catch (Exception e) {}
    	System.out.println(master);
    	if(master != null) return 0;
    	else return 1;
    }
    return 2;
}