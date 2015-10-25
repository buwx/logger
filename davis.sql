CREATE TABLE logger (
  id          INT NOT NULL AUTO_INCREMENT,
  datetime    INT NOT NULL DEFAULT 0,
  sensor      CHAR(1) NOT NULL,
  data        VARCHAR(80),
  description VARCHAR(80),
  PRIMARY KEY (id),
  INDEX (datetime),
  INDEX (sensor)
);

CREATE TABLE persistent (
  id         INT NOT NULL AUTO_INCREMENT,
  last_id    INT,
  last_time  INT,
  PRIMARY KEY (id)
);

INSERT INTO persistent(last_id,last_time) VALUES(0,0);
