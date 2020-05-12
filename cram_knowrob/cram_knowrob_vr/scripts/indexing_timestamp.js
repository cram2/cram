// connect to database

var db = connect('127.0.0.1:27017/Own-Episodes_set-clean-table');

//iterate over all collections and add the timestamp index to them
db.getCollectionNames().forEach(function(collname) {
	if (collname.indexOf('RawData') >= 0){
		print("current collection: " + collname);
		collection = db.getCollection(collname);
		collection.createIndex( {timestamp : 1});
	}
	});

