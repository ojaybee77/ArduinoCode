void fuzzyLogicSetup() {

  // instantiating a Fuzzy object
Fuzzy *fuzzy = new Fuzzy();

// FuzzyInput Membership Functions
FuzzySet *tooCool = new FuzzySet(setPoint-10, setPoint-10, setPoint-4, setPoint-0.5);
FuzzySet *desired = new FuzzySet(setPoint-1.5, setPoint-0.3, setPoint+0.3, setPoint+1.5);
FuzzySet *tooWarm = new FuzzySet(setPoint+0.5, setPoint+4, setPoint+10, setPoint+10);

// FuzzyOutput
FuzzySet *reduceHeat = new FuzzySet(-MAX_NEG_PWM_CHANGE, -MAX_NEG_PWM_CHANGE, -10, -5);
FuzzySet *noChange = new FuzzySet(-10, -5, 5, 10);
FuzzySet *increaseHeat = new FuzzySet(5,10, MAX_POS_PWM_CHANGE, MAX_POS_PWM_CHANGE);
  // FuzzyInput Variable  - must be in setup() 
FuzzyInput *f_temperature = new FuzzyInput(1);
f_temperature->addFuzzySet(tooCool);
f_temperature->addFuzzySet(desired);
f_temperature->addFuzzySet(tooWarm);
fuzzy->addFuzzyInput(f_temperature);

// FuzzyOutput Variable
FuzzyOutput *heater = new FuzzyOutput(1);
heater->addFuzzySet(reduceHeat);
heater->addFuzzySet(noChange);
heater->addFuzzySet(increaseHeat);
fuzzy->addFuzzyOutput(heater);


// Building FuzzyRule "IF temp = tooCool THEN heater = increase"
FuzzyRuleAntecedent *ifTempTooCool = new FuzzyRuleAntecedent();
ifTempTooCool->joinSingle(tooCool);
FuzzyRuleConsequent *thenHeaterIncrease = new FuzzyRuleConsequent();
thenHeaterIncrease->addOutput(increaseHeat);
FuzzyRule *fuzzyRule01 = new FuzzyRule(1, ifTempTooCool, thenHeaterIncrease);
fuzzy->addFuzzyRule(fuzzyRule01);

// Building FuzzyRule "IF temp = desired THEN heater = noChange"
FuzzyRuleAntecedent *ifTempOk = new FuzzyRuleAntecedent();
ifTempOk->joinSingle(desired);
FuzzyRuleConsequent *thenHeaterNoChange = new FuzzyRuleConsequent();
thenHeaterNoChange->addOutput(noChange);
FuzzyRule *fuzzyRule02 = new FuzzyRule(1, ifTempOk, thenHeaterNoChange);
fuzzy->addFuzzyRule(fuzzyRule02);

// Building FuzzyRule "IF temp = desired THEN heater = noChange"
FuzzyRuleAntecedent *ifTempTooWarm = new FuzzyRuleAntecedent();
ifTempTooWarm->joinSingle(tooWarm);
FuzzyRuleConsequent *thenHeaterReduce = new FuzzyRuleConsequent();
thenHeaterReduce->addOutput(reduceHeat);
FuzzyRule *fuzzyRule03 = new FuzzyRule(1, ifTempTooWarm, thenHeaterReduce);
fuzzy->addFuzzyRule(fuzzyRule03);


}