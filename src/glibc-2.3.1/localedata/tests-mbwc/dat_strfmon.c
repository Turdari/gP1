/*
 *  TEST SUITE FOR MB/WC FUNCTIONS IN CLIBRARY
 *
 *	 FILE:	dat_strfmon.c
 *
 *	 STRFMON:  size_t strfmon (char *buf, size_t nbyte, char *fmt, ... );
 */

/*
 *  NOTE:
 *
 *  The buffer size should be enough to contain a string including a
 *  null char.
 *  Returns the number of bytes of the string (NOT including a null char).
 */

TST_STRFMON tst_strfmon_loc [] = {
  {
    { Tstrfmon, TST_LOC_de },
    {
      {
	/* #01 */
	/*inp*/ { 24, "%n %% %i",	     123.00			},
	/*exp*/ { 0,1,23,		     "123,00 EUR % 123,00 EUR"	},
      },
      {
	/* #02 */
	/*inp*/ { 24, "%n %% %i",	     123.00			},
	/*exp*/ { 0,1,23,		     "123,00 EUR % 123,00 EUR"	},
      },
      {
	/* #03 */
	/*inp*/ { 23, "%n %% %i",	     123.00			},
	/*exp*/ { E2BIG,1,-1,	     ""					},
      },
      {
	/* #04 */
	/*inp*/ { 31, "%n|%i",	     1234.561				},
	/*exp*/ { 0,1,25,		     "1.234,56 EUR|1.234,56 EUR"},
      },
      {
	/* #05 */
	/*inp*/ { 33, "%n|%i",	    -1234.561				},
	/*exp*/ { 0,1,27,		     "-1.234,56 EUR|-1.234,56 EUR"},
      },
      {
	/* #06 */
	/*inp*/ { 33, "%13n|%12i",	     1234.561			},
	/*exp*/ { 0,1,26,		     " 1.234,56 EUR|1.234,56 EUR"},
      },
      {
	/* #07 */
	/*inp*/ { 33, "%12n|%12i",	    -1234.561			},
	/*exp*/ { 0,1,27,		     "-1.234,56 EUR|-1.234,56 EUR"},
      },
      {
	/* #08 */
	/*inp*/ { 33, "%#5n|%#5i",	     1234.561			},
	/*exp*/ { 0,1,29,		     "  1.234,56 EUR|  1.234,56 EUR"},
      },
      {
	/* #09 */
	/*inp*/ { 33, "%#5n|%#5i",	    -1234.561			},
	/*exp*/ { 0,1,29,		     "- 1.234,56 EUR|- 1.234,56 EUR"},
      },
      {
	/* #10 */
	/*inp*/ { 33, "%=*#5n|%=*#5i",	 1234.561			},
	/*exp*/ { 0,1,29,		     " *1.234,56 EUR| *1.234,56 EUR"},
      },
      {
	/* #11 */
	/*inp*/ { 33, "%=0#5n|%=0#5i",	-1234.561			},
	/*exp*/ { 0,1,29,		     "-01.234,56 EUR|-01.234,56 EUR"},
      },
      {
	/* #12 */
	/*inp*/ { 33, "%^#5n|%^#5i",	-1234.561			},
	/*exp*/ { 0,1,27,		     "- 1234,56 EUR|- 1234,56 EUR"},
      },
      {
	/* #13 */
	/*inp*/ { 33, "%#5.0n|%#5.0i",	 1234.444			},
	/*exp*/ { 0,1,23,		     "  1.234 EUR|  1.234 EUR"	},
      },
      {
	/* #14 */
	/*inp*/ { 33, "%#5.0n|%#5.4i",	-1234.555			},
	/*exp*/ { 0,1,28,		     "- 1.235 EUR|- 1.234,5550 EUR"},
      },
      {
	/* #15 */
	/*inp*/ { 33, "%(#5n|%!(#5i",	-1234.561			},
	/*exp*/ { 0,1,27,		     "( 1.234,56 EUR)|( 1.234,56)"},
      },
      { is_last: 1 }
    }
  },
  {
    { Tstrfmon, TST_LOC_enUS },
    {
      {
	/* #01 */
	/*inp*/ { 22, "%n %% %i",	     123.00			},
	/*exp*/ { 0,1,20,		     "$123.00 % USD 123.00"	},
      },
      {
	/* #02 */
	/*inp*/ { 21, "%n %% %i",	     123.00			},
	/*exp*/ { 0,1,20,		     "$123.00 % USD 123.00"	},
      },
      {
	/* #03 */
	/*inp*/ { 20, "%n %% %i",	     123.00			},
	/*exp*/ { E2BIG,1,-1,	     ""					},
      },
      {
	/* #04 */
	/*inp*/ { 30, "%n|%i",	     1234.561				},
	/*exp*/ { 0,1,22,		     "$1,234.56|USD 1,234.56"	},
      },
      {
	/* #05 */
	/*inp*/ { 32, "%n|%i",	    -1234.561				},
	/*exp*/ { 0,1,24,		     "-$1,234.56|-USD 1,234.56"	},
      },
      {
	/* #06 */
	/*inp*/ { 30, "%12n|%12i",	     1234.561			},
	/*exp*/ { 0,1,25,		     "   $1,234.56|USD 1,234.56"},
      },
      {
	/* #07 */
	/*inp*/ { 32, "%12n|%12i",	    -1234.561			},
	/*exp*/ { 0,1,26,		     "  -$1,234.56|-USD 1,234.56"},
      },
      {
	/* #08 */
	/*inp*/ { 32, "%#5n|%#5i",	     1234.561			},
	/*exp*/ { 0,1,26,		     " $ 1,234.56| USD  1,234.56"},
      },
      {
	/* #09 */
	/*inp*/ { 32, "%#5n|%#5i",	    -1234.561			},
	/*exp*/ { 0,1,26,		     "-$ 1,234.56|-USD  1,234.56"},
      },
      {
	/* #10 */
	/*inp*/ { 32, "%=*#5n|%=*#5i",	 1234.561			},
	/*exp*/ { 0,1,26,		     " $*1,234.56| USD *1,234.56"},
      },
      {
	/* #11 */
	/*inp*/ { 32, "%=0#5n|%=0#5i",	-1234.561			},
	/*exp*/ { 0,1,26,		     "-$01,234.56|-USD 01,234.56"},
      },
      {
	/* #12 */
	/*inp*/ { 32, "%^#5n|%^#5i",	-1234.561			},
	/*exp*/ { 0,1,24,		     "-$ 1234.56|-USD  1234.56"	},
      },
      {
	/* #13 */
	/*inp*/ { 32, "%#5.0n|%#5.0i",	 1234.444			},
	/*exp*/ { 0,1,20,		     " $ 1,234| USD  1,234"	},
      },
      {
	/* #14 */
	/*inp*/ { 32, "%#5.0n|%#5.4i",	-1234.555			},
	/*exp*/ { 0,1,25,		     "-$ 1,235|-USD  1,234.5550"},
      },
      {
	/* #15 */
	/*inp*/ { 32, "%(#5n|%!(#5i",	-1234.561			},
	/*exp*/ { 0,1,24,		     "($ 1,234.56)|( 1,234.56)"	},
      },
      { is_last: 1 }
    }
  },
  {
    { Tstrfmon, TST_LOC_eucJP },
    {
      {
	/* #01 */
	/*inp*/ { 17, "%n %% %i",	 123.00				   },
	/*exp*/ { 0,1,15,		 "\241\357123 % JPY 123"	   },
      },
      {
	/* #02 */
	/*inp*/ { 16, "%n %% %i",	 123.00				   },
	/*exp*/ { 0,1,15,		 "\241\357123 % JPY 123"	   },
      },
      {
	/* #03 */
	/*inp*/ { 15, "%n %% %i",	 123.00				   },
	/*exp*/ { E2BIG,1,-1,		 ""				   },
      },
      {
	/* #04 */
	/*inp*/ { 30, "%n|%i",		 1234.561			   },
	/*exp*/ { 0,1,17,		 "\241\3571,235|JPY 1,235"	   },
      },
      {
	/* #05 */
	/*inp*/ { 32, "%n|%i",		-1234.561			   },
	/*exp*/ { 0,1,19,		 "\241\357-1,235|JPY -1,235"	   },
      },
      {
	/* #06 */
	/*inp*/ { 32, "%12n|%12i",	 1234.561			   },
	/*exp*/ { 0,1,25,		 "     \241\3571,235|   JPY 1,235" },
      },
      {
	/* #07 */
	/*inp*/ { 32, "%12n|%12i",	-1234.561			   },
	/*exp*/ { 0,1,25,		 "    \241\357-1,235|  JPY -1,235" },
      },
      {
	/* #08 */
	/*inp*/ { 32, "%#5n|%#5i",	 1234.561			   },
	/*exp*/ { 0,1,21,		 " \241\357 1,235| JPY  1,235"	   },
      },
      {
	/* #09 */
	/*inp*/ { 32, "%#5n|%#5i",	-1234.561			   },
	/*exp*/ { 0,1,21,		 "\241\357- 1,235|JPY - 1,235"	   },
      },
      {
	/* #10 */
	/*inp*/ { 32, "%=*#5n|%=*#5i",	 1234.561			   },
	/*exp*/ { 0,1,21,		 " \241\357*1,235| JPY *1,235"	   },
      },
      {
	/* #11 */
	/*inp*/ { 32, "%=0#5n|%=0#5i",	-1234.561			   },
	/*exp*/ { 0,1,21,		 "\241\357-01,235|JPY -01,235"	   },
      },
      {
	/* #12 */
	/*inp*/ { 32, "%^#5n|%^#5i",	-1234.561			   },
	/*exp*/ { 0,1,19,		 "\241\357- 1235|JPY - 1235"	   },
      },
      {
	/* #13 */
	/*inp*/ { 32, "%#5.0n|%#5.0i",	 1234.444			   },
	/*exp*/ { 0,1,21,		 " \241\357 1,234| JPY  1,234"	   },
      },
      {
	/* #14 */
	/*inp*/ { 32, "%#5.0n|%#5.4i",	-1234.555			   },
	/*exp*/ { 0,1,26,		 "\241\357- 1,235|JPY - 1,234.5550"},
      },
      {
	/* #15 */
	/*inp*/ { 32, "%(#5n|%!(#5i",	-1234.561			   },
	/*exp*/ { 0,1,19,		 "(\241\357 1,235)|( 1,235)"	   },
      },
      { is_last: 1 }
    }
  },
  {
    { Tstrfmon, TST_LOC_end }
  }
};
