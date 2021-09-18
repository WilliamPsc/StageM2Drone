public void readVar() throws SQLException {
	int n = 0;
	while(n < nbRep){
		if(typeReg == 'W')readAddrW();
		else if(typeReg == 'M')readAddrM();

		if((n + 1) != nbRep) {
			try {
				Thread.sleep(timer) ;
			}  catch (InterruptedException e) {}
		}
		n++;
	}
	String tmp = "";
	if(typeReg == 'W')tmp = "MW";
	else if(typeReg == 'M') tmp = "M";
	requete.select("*", "WHERE registerName = '" + tmp + addrReg + "'");
}